#!/usr/bin/env python3
"""
Mouse -> /probe/pose (geometry_msgs/msg/PoseStamped)
       + Robotiq gripper through the Robotiq URCap socket (port 63352)

  Normal mode:
    Mouse left/right      -> pose.position.x
    Mouse up/down         -> pose.position.y
    Mouse wheel           -> pose.position.z

  Gripper (works in any mode):
    Hold LEFT button      -> gripper closes in ONE continuous motion.
    Hold RIGHT button     -> gripper opens in ONE continuous motion.
    Release               -> gripper stops where it is (no step back).
                             If it is squeezing an object, it KEEPS squeezing.
    Speed: starts slow (precise), and after `grip_fine_time` seconds of
    holding it accelerates to `grip_fast_speed` over `grip_accel_time` s.
    Shift + click         -> stays slow the whole time (precision mode).

  While holding the rotate key (parameter `rotate_key`, default left Ctrl):
    Position is frozen.
    Drag in any direction -> the tool's Z axis tilts towards that direction.
    Mouse wheel           -> spin the tool about its own Z axis
    Release the key       -> the orientation is KEPT.

  Keys:  Space = call a ROS 2 service (name/type are parameters)
         R = reset Z,  O = reset orientation,  C = re-centre XY on the cursor,
         Esc / Q = quit

Requirements on the robot:
  The Robotiq Grippers URCap must be installed and the gripper connected
  through it. The URCap opens a text socket on port 63352.

Run:
  source /opt/ros/<distro>/setup.bash
  python3 mouse_pose_publisher_urcap.py --ros-args -p robot_ip:=192.168.0.20

Optional parameters:
  -p topic:=/probe/pose -p frame_id:=world
  -p xy_scale:=0.001 -p z_step:=0.005 -p rot_scale:=0.005 -p spin_step:=0.05
  -p rotate_key:=ctrl_l       pynput Key name (ctrl_l, ctrl_r, alt_l...) or one char
  -p close_button:=1 -p open_button:=3
  -p robot_ip:=192.168.0.20 -p gripper_port:=63352
  -p grip_slow_speed:=0        0..255, speed at the start of a press
  -p grip_fast_speed:=255      0..255, speed after accelerating
  -p grip_fine_time:=0.4       s held at slow speed before accelerating
  -p grip_accel_time:=1.0      s to go from slow to fast speed
  -p grip_force:=150           0..255
  -p grip_stop_lead:=0.06      s of motion anticipated when stopping
                               (raise it if the gripper still steps back)
  -p grip_open_position:=0     0..255  (0 = fully open)
  -p grip_closed_position:=255 0..255  (255 = fully closed)
  -p rate_hz:=50.0
  -p service_name:=/teleop_manipulators_node/clutch
  -p service_type:=std_srvs/srv/SetBool   pkg/srv/Type (or pkg/Type)
                                           Space toggles True/False (if .data exists)

  Buttons on Linux: 1 = left, 2 = middle (wheel press), 3 = right.
"""

import math
import queue
import socket
import threading
import time
import tkinter as tk
from importlib import import_module
from pynput import keyboard

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


BUTTON_NAMES = {1: "left", 2: "middle", 3: "right"}
SHIFT_MASK = 0x0001  # Tk event.state bit for Shift


def axis_angle_quat(ax, ay, az, angle):
    """Quaternion (x, y, z, w) for a rotation of `angle` rad about unit axis (ax, ay, az)."""
    s = math.sin(angle / 2)
    return (ax * s, ay * s, az * s, math.cos(angle / 2))


def quat_mul(a, b):
    """Hamilton product a * b, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def clamp255(v):
    return int(round(max(0, min(255, v))))


class RobotiqURCapGripper(threading.Thread):
    """Talks to the Robotiq URCap socket in a background thread so the GUI
    never blocks. The GUI queues commands:
        ("move", position, speed)  -> go to position at that speed
        ("stop",)                  -> stop where the gripper is right now
    When there is nothing to send, the thread polls the actual position."""

    def __init__(self, host, port, force, stop_lead, logger):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.force = clamp255(force)
        self.stop_lead = max(0.0, float(stop_lead))
        self.log = logger
        self.holding = False  # True when stopped on an object
        self._settle = None  # (direction, target, until) after a stop

        self.cmds = queue.Queue()
        self.actual = None  # last position read from the gripper (0..255)
        self.status = "connecting"
        self.running = True
        self.sock = None
        self._sent_pos = None
        self._sent_spe = None

    # ---- called from the GUI thread ----
    def move(self, pos, speed):
        self.cmds.put(("move", clamp255(pos), clamp255(speed)))

    def stop_here(self, direction):
        """direction: +1 if the gripper was moving towards 255, -1 towards 0."""
        self.cmds.put(("stop", direction))

    def shutdown(self):
        self.running = False

    # ---- socket helpers (worker thread only) ----
    def _cmd(self, c):
        self.sock.sendall((c + "\n").encode())
        return self.sock.recv(1024).decode(errors="ignore").strip()

    def _get_int(self, var):
        """Send 'GET XXX', parse the reply 'XXX n' and return n."""
        reply = self._cmd(f"GET {var}")
        try:
            return int(reply.split()[-1])
        except (ValueError, IndexError):
            return None

    def _connect(self):
        self.status = "connecting"
        self.sock = socket.create_connection((self.host, self.port), timeout=2.0)

        # Only activate if not already active (activation makes the fingers move)
        if self._get_int("STA") != 3:
            self.status = "activating"
            self.log.info("Activating gripper...")
            self._cmd("SET ACT 1")
            t0 = time.monotonic()
            while self._get_int("STA") != 3:
                if time.monotonic() - t0 > 10.0:
                    raise OSError("activation timed out")
                time.sleep(0.2)

        self._cmd(f"SET FOR {self.force}")
        self.actual = self._get_int("POS")
        # Hold the current position so nothing moves when GTO is set
        if self.actual is not None:
            self._cmd(f"SET POS {self.actual}")
            self._sent_pos = self.actual
        self._cmd("SET GTO 1")
        self._sent_spe = None
        self.status = "ready"
        self.log.info(f"Gripper ready on {self.host}:{self.port} (pos {self.actual})")

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _send_move(self, pos, spe):
        self.holding = False
        self._settle = None
        changed_spe = spe != self._sent_spe
        if changed_spe:
            self._cmd(f"SET SPE {spe}")
            self._sent_spe = spe
        # Re-send POS after a speed change so the new speed is applied mid-motion
        if changed_spe or pos != self._sent_pos:
            self._cmd(f"SET POS {pos}")
            self._sent_pos = pos

    def _send_stop(self, direction):
        # gOBJ: 0 moving, 1 stopped on object while opening,
        #       2 stopped on object while closing, 3 at requested position
        obj = self._get_int("OBJ")
        if obj in (1, 2):
            # Blocked by an object: keep the current target so the grip keeps
            # its force (re-targeting here is what made it loosen / step back)
            self.holding = True
            return

        # Measure the speed with two quick reads, then aim slightly ahead,
        # because the fingers keep moving while the command travels.
        t1 = time.monotonic()
        p1 = self._get_int("POS")
        t2 = time.monotonic()
        p2 = self._get_int("POS")
        if p1 is None or p2 is None:
            return
        self.actual = p2
        dt = max(1e-3, t2 - t1)
        v_along = max(0.0, (p2 - p1) / dt * direction)  # counts/s in motion direction
        target = p2 + direction * (v_along * self.stop_lead + (1 if v_along > 0 else 0))
        target = clamp255(target)
        self._cmd(f"SET POS {target}")
        self._sent_pos = target
        self._settle = (direction, target, time.monotonic() + 0.4)

    def run(self):
        while self.running:
            try:
                if self.sock is None:
                    self._connect()

                # Drain everything the GUI queued, keeping only what matters:
                # the latest move, and any stop that came after it.
                pending_move = None
                did_something = False
                while True:
                    try:
                        item = self.cmds.get_nowait()
                    except queue.Empty:
                        break
                    if item[0] == "move":
                        pending_move = item
                    else:  # stop
                        if pending_move is not None:
                            self._send_move(pending_move[1], pending_move[2])
                            pending_move = None
                        self._send_stop(item[1])
                        did_something = True
                if pending_move is not None:
                    self._send_move(pending_move[1], pending_move[2])
                    did_something = True

                if not did_something:
                    pos = self._get_int("POS")
                    if pos is not None:
                        self.actual = pos
                        # Just after a stop: if the fingers went past the target,
                        # move the target forward instead of letting them come back
                        if self._settle is not None:
                            d, tgt, until = self._settle
                            if time.monotonic() > until:
                                self._settle = None
                            elif (pos - tgt) * d > 1:
                                new_t = clamp255(pos + d)
                                self._cmd(f"SET POS {new_t}")
                                self._sent_pos = new_t
                                self._settle = (d, new_t, until)

                time.sleep(0.02)

            except (OSError, socket.timeout) as e:
                self.status = f"disconnected ({e})"
                self.log.warn(f"Gripper socket error: {e}; retrying in 2 s")
                self._close()
                # Drop stale commands so the gripper doesn't jump on reconnect
                while not self.cmds.empty():
                    try:
                        self.cmds.get_nowait()
                    except queue.Empty:
                        break
                time.sleep(2.0)

        self._close()


class MousePosePublisher(Node):
    def __init__(self):
        super().__init__("mouse_pose_publisher")

        # ---- pose parameters ----
        self.declare_parameter("topic", "/probe/pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("xy_scale", 0.001)  # metres per pixel
        self.declare_parameter("z_step", 0.003)  # metres per wheel click
        self.declare_parameter("rot_scale", 0.005)  # radians per pixel while rotating
        self.declare_parameter(
            "spin_step", 0.05
        )  # radians per wheel click while rotating
        self.declare_parameter("rotate_key", "ctrl_l")
        self.declare_parameter("service_name", "/teleop_manipulators_node/clutch")
        self.declare_parameter("service_type", "std_srvs/srv/SetBool")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)

        # ---- gripper parameters ----
        self.declare_parameter("close_button", 1)  # left
        self.declare_parameter("open_button", 3)  # right
        self.declare_parameter("robot_ip", "192.168.0.20")
        self.declare_parameter("gripper_port", 63352)
        self.declare_parameter("grip_slow_speed", 0)
        self.declare_parameter("grip_fast_speed", 255)
        self.declare_parameter("grip_fine_time", 0.4)
        self.declare_parameter("grip_accel_time", 1.0)
        self.declare_parameter("grip_force", 150)
        self.declare_parameter("grip_open_position", 0)
        self.declare_parameter("grip_closed_position", 255)
        self.declare_parameter("grip_stop_lead", 0.06)  # s anticipated when stopping

        self.rotate_key_name = str(self.get_parameter("rotate_key").value)
        self.rotating_key = getattr(keyboard.Key, self.rotate_key_name, None)
        if self.rotating_key is None:
            # Not a pynput Key member: treat it as a single character
            self.rotating_key = self.rotate_key_name

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.xy_scale = float(self.get_parameter("xy_scale").value)
        self.z_step = float(self.get_parameter("z_step").value)
        self.rot_scale = float(self.get_parameter("rot_scale").value)
        self.spin_step = float(self.get_parameter("spin_step").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)

        self.close_button = int(self.get_parameter("close_button").value)
        self.open_button = int(self.get_parameter("open_button").value)
        robot_ip = self.get_parameter("robot_ip").value
        port = int(self.get_parameter("gripper_port").value)
        self.slow_spe = clamp255(self.get_parameter("grip_slow_speed").value)
        self.fast_spe = clamp255(self.get_parameter("grip_fast_speed").value)
        self.fine_time = max(0.0, float(self.get_parameter("grip_fine_time").value))
        self.accel_time = max(0.01, float(self.get_parameter("grip_accel_time").value))
        self.grip_open = clamp255(self.get_parameter("grip_open_position").value)
        self.grip_closed = clamp255(self.get_parameter("grip_closed_position").value)

        buttons = [self.close_button, self.open_button]
        if len(set(buttons)) != 2:
            self.get_logger().warn(
                f"close/open buttons overlap ({buttons}); behaviour will be odd."
            )

        # ---- pose publisher ----
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_pose)

        # ---- Space -> ROS 2 service ----
        self.service_name = str(self.get_parameter("service_name").value)
        self.service_type = str(self.get_parameter("service_type").value)
        self.service_client = self._make_service_client()
        self._service_down = False
        self._service_pressed = False
        self._service_bool = True  # first Space press sends True, then alternates

        # ---- gripper (URCap socket in a background thread) ----
        self.gripper = RobotiqURCapGripper(
            robot_ip,
            port,
            self.get_parameter("grip_force").value,
            self.get_parameter("grip_stop_lead").value,
            self.get_logger(),
        )
        self._nudge = False
        self.gripper.start()

        self.held = []  # gripper buttons held, most recent last: "close"/"open"
        self.press_t = 0.0
        self.fine_mode = False
        self.grip_spe = 0  # speed last requested
        self._last_spe_send = 0.0
        self._shown_grip = None
        self.grip_timer = self.create_timer(0.02, self.update_gripper)

        # Position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.off_x = 0.0
        self.off_y = 0.0

        # Orientation
        self.rotating = False
        self.press_px = (0, 0)
        self.drag = (0.0, 0.0)
        self.spin_angle = 0.0
        self.base_q = (0.0, 0.0, 0.0, 1.0)

        self.last_cursor = (self.w / 2, self.h / 2)
        self.rot_key = False

        self._build_window()

        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

        self.get_logger().info(f"Publishing PoseStamped on {topic} at {rate_hz} Hz")
        self.get_logger().info(f"Gripper via URCap socket {robot_ip}:{port}")

    def _is_rotate_key(self, key):
        if isinstance(self.rotating_key, str):
            return getattr(key, "char", None) == self.rotating_key
        return key == self.rotating_key

    def on_press(self, key):
        if self._is_rotate_key(key):
            self.rot_key = True
        elif key == keyboard.Key.space:
            if not self._service_down:
                self._service_down = True
                self._service_pressed = True
        elif key == keyboard.Key.esc:
            return False

    def on_release(self, key):
        if self._is_rotate_key(key):
            self.rot_key = False
        elif key == keyboard.Key.space:
            self._service_down = False

    # ---------------- GUI ----------------
    def _build_window(self):
        self.root = tk.Tk()
        self.root.title("Mouse -> /probe/pose + gripper (URCap)")
        self.canvas = tk.Canvas(
            self.root,
            width=self.w,
            height=self.h,
            bg="#1e1e1e",
            highlightthickness=0,
            cursor="crosshair",
        )

        controls = tk.Frame(self.root)
        controls.pack(side="bottom", fill="x")
        self._xy_var = tk.DoubleVar(value=self.xy_scale)
        self._z_var = tk.DoubleVar(value=self.z_step)
        self._grip_var = tk.DoubleVar(value=self.fast_spe)
        for var, lo, hi, res, label, cb in (
            (self._xy_var, 0.0001, 0.01, 0.0001, "XY m/px", self._on_xy_scale),
            (self._z_var, 0.0005, 0.05, 0.0005, "Z m/click", self._on_z_step),
            (self._grip_var, 0, 255, 1, "Gripper speed (0-255)", self._on_grip_speed),
        ):
            tk.Scale(
                controls,
                from_=lo,
                to=hi,
                resolution=res,
                orient="horizontal",
                label=label,
                variable=var,
                command=cb,
                length=320,
            ).pack(side="left", expand=True, fill="x")

        self.canvas.pack(fill="both", expand=True)

        cx, cy = self.w / 2, self.h / 2
        self.canvas.create_line(cx, 0, cx, self.h, fill="#444")
        self.canvas.create_line(0, cy, self.w, cy, fill="#444")
        self.label = self.canvas.create_text(
            10, 10, anchor="nw", fill="#ddd", font=("Monospace", 12), text=""
        )
        self.dot = self.canvas.create_oval(
            cx - 5, cy - 5, cx + 5, cy + 5, fill="#4fc3f7", outline=""
        )

        self.canvas.bind("<Motion>", self.on_motion)
        for b in (1, 2, 3):
            self.canvas.bind(f"<B{b}-Motion>", self.on_motion)

        cb, ob = self.close_button, self.open_button
        self.canvas.bind(
            f"<ButtonPress-{cb}>", lambda e: self.on_grip_press("close", e)
        )
        self.canvas.bind(
            f"<ButtonRelease-{cb}>", lambda e: self.on_grip_release("close")
        )
        self.canvas.bind(f"<ButtonPress-{ob}>", lambda e: self.on_grip_press("open", e))
        self.canvas.bind(
            f"<ButtonRelease-{ob}>", lambda e: self.on_grip_release("open")
        )

        self.canvas.bind("<Button-4>", lambda e: self.on_wheel(+1))
        self.canvas.bind("<Button-5>", lambda e: self.on_wheel(-1))
        self.canvas.bind(
            "<MouseWheel>", lambda e: self.on_wheel(1 if e.delta > 0 else -1)
        )

        self.root.bind("<Key-r>", lambda e: self.reset_z())
        self.root.bind("<Key-R>", lambda e: self.reset_z())
        self.root.bind("<Key-o>", lambda e: self.reset_orientation())
        self.root.bind("<Key-O>", lambda e: self.reset_orientation())
        self.root.bind("<Key-c>", lambda e: self.recentre())
        self.root.bind("<Key-C>", lambda e: self.recentre())
        self.root.bind("<Escape>", lambda e: self.quit())
        self.root.bind("<Key-q>", lambda e: self.quit())
        self.root.protocol("WM_DELETE_WINDOW", self.quit)

        self.running = True
        self._update_label()

    def _cursor_to_xy(self, px, py):
        cx = self.canvas.winfo_width() / 2
        cy = self.canvas.winfo_height() / 2
        return (px - cx) * self.xy_scale, -(py - cy) * self.xy_scale

    def _on_xy_scale(self, value):
        self.xy_scale = float(value)

    def _on_z_step(self, value):
        self.z_step = float(value)

    def _on_grip_speed(self, value):
        self.fast_spe = clamp255(float(value))
        # Apply immediately while a grip button is held (timer also catches it)
        if self.held and time.monotonic() - self._last_spe_send >= 0.1:
            self._start_motion()

    def begin_rotate(self):
        if self.rotating:
            return
        self.rotating = True
        self.press_px = self.last_cursor
        self.canvas.itemconfig(self.dot, fill="#ff8a65")
        self.canvas.config(cursor="exchange")
        self._update_label()

    def end_rotate(self):
        if not self.rotating:
            return
        self.rotating = False
        self.base_q = self.orientation()
        self.drag = (0.0, 0.0)
        self.spin_angle = 0.0
        px, py = self.last_cursor
        cx_m, cy_m = self._cursor_to_xy(px, py)
        self.off_x = self.x - cx_m
        self.off_y = self.y - cy_m
        self.canvas.coords(self.dot, px - 5, py - 5, px + 5, py + 5)
        self.canvas.itemconfig(self.dot, fill="#4fc3f7")
        self.canvas.config(cursor="crosshair")
        self._update_label()

    def on_motion(self, event):
        self.last_cursor = (event.x, event.y)
        if self.rotating:
            dx = event.x - self.press_px[0]
            dy = -(event.y - self.press_px[1])
            self.drag = (dx, dy)
        else:
            cx_m, cy_m = self._cursor_to_xy(event.x, event.y)
            self.x = cx_m + self.off_x
            self.y = cy_m + self.off_y
            self.canvas.coords(
                self.dot, event.x - 5, event.y - 5, event.x + 5, event.y + 5
            )
        self._update_label()

    def on_wheel(self, direction):
        if self.rotating:
            self.spin_angle += direction * self.spin_step
        else:
            self.z += direction * self.z_step
        self._update_label()

    def reset_orientation(self):
        self.base_q = (0.0, 0.0, 0.0, 1.0)
        self.drag = (0.0, 0.0)
        self.spin_angle = 0.0
        if self.rotating:
            self.press_px = self.last_cursor
        self._update_label()

    def reset_z(self):
        self.z = 0.0
        self._update_label()

    def recentre(self):
        self.off_x = self.off_y = 0.0
        self.x, self.y = self._cursor_to_xy(*self.last_cursor)
        self._update_label()

    def orientation(self):
        dx, dy = self.drag
        dist = math.hypot(dx, dy)
        if dist > 1e-9:
            q_tilt = axis_angle_quat(dx / dist, dy / dist, 0.0, dist * self.rot_scale)
        else:
            q_tilt = (0.0, 0.0, 0.0, 1.0)
        q_spin = axis_angle_quat(0.0, 0.0, 1.0, self.spin_angle)
        return quat_mul(quat_mul(q_tilt, self.base_q), q_spin)

    # ---------------- gripper ----------------
    def _speed_now(self):
        """Slow at first (precision), then accelerate the longer the button is held."""
        if self.fine_mode:
            return self.slow_spe
        held = time.monotonic() - self.press_t - self.fine_time
        if held <= 0:
            return self.slow_spe
        frac = min(1.0, held / self.accel_time)
        return clamp255(self.slow_spe + frac * (self.fast_spe - self.slow_spe))

    def _start_motion(self):
        """Send ONE move to the end position; the gripper moves smoothly by itself."""
        action = self.held[-1]
        target = self.grip_closed if action == "close" else self.grip_open
        # The firmware only applies a new speed together with a new target,
        # so alternate the target by one count every time we resend.
        self._nudge = not self._nudge
        if self._nudge:
            target -= self._dir(action)
        self.grip_spe = self._speed_now()
        self.gripper.move(target, self.grip_spe)
        self._last_spe_send = time.monotonic()

    def _dir(self, action):
        """+1 if this action moves towards 255, -1 if towards 0."""
        close_sign = 1 if self.grip_closed >= self.grip_open else -1
        return close_sign if action == "close" else -close_sign

    def on_grip_press(self, action, event):
        if action in self.held:
            return
        self.held.append(action)
        self.press_t = time.monotonic()
        self.fine_mode = bool(event.state & SHIFT_MASK)
        self._start_motion()
        self._update_label()

    def on_grip_release(self, action):
        if action in self.held:
            self.held.remove(action)
        if self.held:
            # The other button is still held: continue in its direction
            self.press_t = time.monotonic()
            self._start_motion()
        else:
            self.gripper.stop_here(self._dir(action))
        self._update_label()

    def update_gripper(self):
        if self.held:
            spe = self._speed_now()
            # Only resend when the speed changed noticeably, at most 10x per second
            now = time.monotonic()
            if abs(spe - self.grip_spe) >= 8 and now - self._last_spe_send >= 0.1:
                self._start_motion()
                self._update_label()

        shown = (self.gripper.status, self.gripper.actual, self.gripper.holding)
        if shown != self._shown_grip:
            self._update_label()

    def _grip_bar(self, p, width=20):
        span = self.grip_closed - self.grip_open
        frac = 0.0 if span == 0 else (p - self.grip_open) / span
        frac = min(1.0, max(0.0, frac))
        n = int(round(frac * width))
        return "[" + "#" * n + "-" * (width - n) + f"] {frac * 100:5.1f}% closed"

    # ---------------- label ----------------
    def _update_label(self):
        mode = "ROTATING" if self.rotating else "position"
        qx, qy, qz, qw = self.orientation()
        zx = 2 * (qx * qz + qw * qy)
        zy = 2 * (qy * qz - qw * qx)
        zz = 1 - 2 * (qx * qx + qy * qy)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, zz))))
        direction = math.degrees(math.atan2(zy, zx)) if tilt > 1e-6 else 0.0

        if self.held:
            grip_state = "CLOSING" if self.held[-1] == "close" else "OPENING"
            grip_state += f"  speed {self.grip_spe:3d}"
            if self.fine_mode:
                grip_state += " (fine)"
        elif self.gripper.holding:
            grip_state = "HOLDING object"
        else:
            grip_state = "idle"
        status = self.gripper.status
        actual = self.gripper.actual
        self._shown_grip = (status, actual, self.gripper.holding)
        actual_txt = (
            "n/a" if actual is None else f"{actual:3d} {self._grip_bar(actual)}"
        )

        cb = BUTTON_NAMES.get(self.close_button, str(self.close_button))
        ob = BUTTON_NAMES.get(self.open_button, str(self.open_button))

        self.canvas.itemconfig(
            self.label,
            text=(
                f"mode    : {mode}\n"
                f"x       : {self.x:+.4f} m\n"
                f"y       : {self.y:+.4f} m\n"
                f"z       : {self.z:+.4f} m\n"
                f"tilt    : {tilt:7.2f} deg  towards {direction:+7.1f} deg\n"
                f"quat    : x={qx:+.3f} y={qy:+.3f} z={qz:+.3f} w={qw:+.3f}\n"
                f"gripper : {grip_state}   socket: {status}\n"
                f"  pos   : {actual_txt}\n\n"
                f"hold {cb} = close gripper   hold {ob} = open gripper\n"
                f"  (slow first, then faster; Shift+click = always slow)\n"
                f"hold {self.rotate_key_name} = rotate   Space = {self.service_name}\n"
                f"wheel = z (spin while rotating)\n"
                f"R = reset z   O = reset orientation\n"
                f"C = re-centre   Esc = quit"
            ),
        )

    # ---------------- ROS ----------------
    def _make_service_client(self):
        try:
            parts = self.service_type.split("/")
            if len(parts) == 3:  # pkg/srv/Type
                pkg, _, srv = parts
            elif len(parts) == 2:  # pkg/Type
                pkg, srv = parts
            else:
                raise ValueError("expected pkg/srv/Type")
            srv_cls = getattr(import_module(f"{pkg}.srv"), srv)
            return self.create_client(srv_cls, self.service_name)
        except Exception as e:
            self.get_logger().warn(
                f"Service disabled: cannot load {self.service_type!r} "
                f"for {self.service_name!r}: {e}"
            )
            return None

    def call_service(self):
        client = self.service_client
        if client is None:
            self.get_logger().warn("No service configured")
            return
        if not client.service_is_ready():
            self.get_logger().warn(
                f"Service {self.service_name} not available",
                throttle_duration_sec=2.0,
            )
            return
        req = client.srv_type.Request()
        if hasattr(req, "data"):
            req.data = self._service_bool
            self._service_bool = not self._service_bool
        client.call_async(req).add_done_callback(self._service_done)

    def _service_done(self, future):
        try:
            self.get_logger().info(f"Service {self.service_name} -> {future.result()}")
        except Exception as e:
            self.get_logger().warn(f"Service {self.service_name} failed: {e}")

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = -self.y
        msg.pose.position.y = self.x
        msg.pose.position.z = self.z
        qx, qy, qz, qw = self.orientation()
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)

    # ---------------- main loop ----------------
    def spin(self):
        def tick():
            if not self.running:
                return
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.rot_key:
                self.begin_rotate()
            else:
                self.end_rotate()
            if self._service_pressed:
                self._service_pressed = False
                self.call_service()
            self.root.after(5, tick)

        tick()
        self.root.mainloop()

    def quit(self):
        self.running = False
        self.listener.stop()
        self.gripper.shutdown()
        self.root.quit()


def main():
    rclpy.init()
    node = MousePosePublisher()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "listener", None) is not None:
            node.listener.stop()
        node.gripper.shutdown()
        node.root.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
