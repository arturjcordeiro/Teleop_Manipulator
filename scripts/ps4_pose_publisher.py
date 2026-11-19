#!/usr/bin/env python3
"""
PS4 controller -> /probe/pose (geometry_msgs/msg/PoseStamped)
               + Robotiq gripper through the Robotiq URCap socket (port 63352)

  Velocity control: hold a stick and the pose keeps moving; let go and it stops.

  Motion:
    Left stick            -> move in X / Y
    D-pad up / down       -> move in Z
    Right stick           -> tilt the tool's Z axis
    L1 / R1               -> spin the tool about its own Z axis
    Hold L3               -> precision mode (all motions multiplied by fine_scale)

  Gripper:
    R2 (analog)           -> close,  L2 (analog) -> open
                             The deeper the trigger, the faster the gripper moves.
    Release the trigger   -> gripper stops where it is (no step back).
                             If it is squeezing an object, it KEEPS squeezing.

  Buttons:
    Cross (X) -> call a ROS 2 service (name/type are parameters)
    Square    -> reset Z
    Triangle  -> reset orientation
    Circle    -> reset X / Y to 0
    Options   -> quit   (Esc / Q also work when the status window has focus)

  Sliders (separate window):
    X/Y, Z, tilt and spin speeds + gripper max speed, applied live.

Requirements:
  pip install pygame   (version 2.x; uses SDL's game-controller mapping)
  PS4 controller on USB or paired over Bluetooth.
  The Robotiq Grippers URCap must be installed on the robot (socket on 63352).

Run:
  source /opt/ros/<distro>/setup.bash
  python3 ps4_pose_publisher_urcap.py --ros-args -p robot_ip:=192.168.0.20

Optional parameters:
  -p topic:=/probe/pose -p frame_id:=world -p rate_hz:=50.0
  -p lin_speed:=0.10        m/s at full stick (X / Y)
  -p z_speed:=0.05          m/s with the D-pad (Z)
  -p rot_speed:=0.8         rad/s at full right stick (tilt)
  -p spin_speed:=1.0        rad/s with L1 / R1 (spin)
  -p fine_scale:=0.25       speed factor while Cross is held
  -p deadzone:=0.12         stick dead zone (0..1)
  -p trigger_threshold:=0.05
  -p xy_limit:=0.4 -p z_limit:=0.5   workspace clamp, metres (+/-)
  -p robot_ip:=192.168.0.20 -p gripper_port:=63352
  -p grip_slow_speed:=0 -p grip_fast_speed:=255   0..255, light / full trigger
  -p grip_force:=150 -p grip_stop_lead:=0.06
  -p grip_open_position:=0 -p grip_closed_position:=255
  -p service_name:=/teleop_manipulators_node/clutch
  -p service_type:=std_srvs/srv/SetBool   pkg/srv/Type (or pkg/Type)
                                           Cross toggles True/False (if .data exists)
"""

import math
import os
import queue
import socket
import threading
import time
import tkinter as tk
from importlib import import_module

# Keep reading the controller even when the status window is not focused
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame
from pygame._sdl2 import controller as sdl_controller

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


AXIS_MAX = 32767.0


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


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def clamp255(v):
    return int(round(clamp(v, 0, 255)))


def radial_deadzone(x, y, dz):
    """Circular dead zone, rescaled so output still goes smoothly from 0 to 1."""
    mag = math.hypot(x, y)
    if mag <= dz:
        return 0.0, 0.0
    scale = min(1.0, (mag - dz) / (1.0 - dz)) / mag
    return x * scale, y * scale


class RobotiqURCapGripper(threading.Thread):
    """Talks to the Robotiq URCap socket in a background thread so the control
    loop never blocks. Commands are queued:
        ("move", position, speed)  -> go to position at that speed
        ("stop", direction)        -> stop where the gripper is right now
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

    # ---- called from the control thread ----
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
            # Blocked by an object: keep the current target so the grip keeps its force
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
        v_along = max(0.0, (p2 - p1) / dt * direction)
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

                # Drain the queue, keeping the latest move and any stop after it
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
                self.log.warning(f"Gripper socket error: {e}; retrying in 2 s")
                self._close()
                # Drop stale commands so the gripper doesn't jump on reconnect
                while not self.cmds.empty():
                    try:
                        self.cmds.get_nowait()
                    except queue.Empty:
                        break
                time.sleep(2.0)

        self._close()


class PS4PosePublisher(Node):
    def __init__(self):
        super().__init__("ps4_pose_publisher")

        def param(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).value

        # ---- pose parameters ----
        topic = param("topic", "/probe/pose")
        self.frame_id = param("frame_id", "world")
        rate_hz = float(param("rate_hz", 120.0))
        self.lin_speed = float(param("lin_speed", 0.10))
        self.z_speed = float(param("z_speed", 0.05))
        self.rot_speed = float(param("rot_speed", 0.8))
        self.spin_speed = float(param("spin_speed", 1.0))
        self.fine_scale = float(param("fine_scale", 0.25))
        self.deadzone = clamp(float(param("deadzone", 0.12)), 0.0, 0.9)
        self.trig_thr = clamp(float(param("trigger_threshold", 0.05)), 0.0, 0.9)
        self.xy_limit = abs(float(param("xy_limit", 0.4)))
        self.z_limit = abs(float(param("z_limit", 0.5)))

        # ---- gripper parameters ----
        robot_ip = param("robot_ip", "192.168.0.20")
        port = int(param("gripper_port", 63352))
        self.slow_spe = clamp255(param("grip_slow_speed", 0))
        self.fast_spe = clamp255(param("grip_fast_speed", 255))
        grip_force = param("grip_force", 150)
        grip_stop_lead = param("grip_stop_lead", 0.06)
        self.grip_open = clamp255(param("grip_open_position", 0))
        self.grip_closed = clamp255(param("grip_closed_position", 255))

        # ---- service parameters ----
        self.service_name = str(
            param("service_name", "/teleop_manipulators_node/clutch")
        )
        self.service_type = str(param("service_type", "std_srvs/srv/SetBool"))

        # ---- state ----
        self.x = self.y = self.z = 0.0
        self.q = (0.0, 0.0, 0.0, 1.0)
        self.fine = False
        self.pad = None
        self.pad_name = None
        self._last_scan = 0.0
        self._prev_buttons = {}
        self.grip_action = None  # None / "close" / "open"
        self.grip_spe = 0
        self._last_spe_send = 0.0
        self._nudge = False
        self.running = True
        self._frame = 0
        self._last_tick = time.monotonic()
        self.draw_every = max(1, int(round(rate_hz / 15)))  # ~15 fps status window

        # ---- pygame / controller ----
        pygame.init()
        sdl_controller.init()
        self.screen = pygame.display.set_mode((620, 380))
        pygame.display.set_caption("PS4 -> /probe/pose + gripper (URCap)")
        self.font = pygame.font.SysFont("monospace", 15)

        # ---- ROS ----
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        # ---- Cross -> ROS 2 service ----
        self.service_client = self._make_service_client()
        self._service_bool = True  # first Cross press sends True, then alternates

        # ---- live sensitivity sliders ----
        threading.Thread(target=self._slider_window, daemon=True).start()

        # ---- gripper thread ----
        self.gripper = RobotiqURCapGripper(
            robot_ip, port, grip_force, grip_stop_lead, self.get_logger()
        )
        self.gripper.start()

        self.get_logger().info(f"Publishing PoseStamped on {topic} at {rate_hz} Hz")
        self.get_logger().info(f"Gripper via URCap socket {robot_ip}:{port}")
        self.get_logger().info("Waiting for a PS4 controller...")

    # ---------------- controller ----------------
    def _ensure_pad(self):
        """Return True if a controller is connected; (re)connect if needed."""
        if self.pad is not None:
            try:
                if self.pad.attached():
                    return True
            except pygame.error:
                pass
            self.get_logger().warning("Controller disconnected")
            self._on_pad_lost()
            try:
                self.pad.quit()
            except pygame.error:
                pass
            self.pad = None

        now = time.monotonic()
        if now - self._last_scan < 1.0:
            return False
        self._last_scan = now

        for i in range(sdl_controller.get_count()):
            if sdl_controller.is_controller(i):
                try:
                    self.pad = sdl_controller.Controller(i)
                except pygame.error:
                    continue
                try:
                    self.pad_name = self.pad.as_joystick().get_name()
                except Exception:
                    self.pad_name = f"controller {i}"
                self._prev_buttons = {}
                self.get_logger().info(f"Controller connected: {self.pad_name}")
                return True
        return False

    def _on_pad_lost(self):
        # Safety: stop the gripper if it was moving
        self._update_gripper(None, 0.0)

    def _axis(self, axis):
        """Stick axis in -1..1."""
        return clamp(self.pad.get_axis(axis) / AXIS_MAX, -1.0, 1.0)

    def _trigger(self, axis):
        """Trigger axis in 0..1."""
        return clamp(self.pad.get_axis(axis) / AXIS_MAX, 0.0, 1.0)

    def _btn(self, button):
        return 1 if self.pad.get_button(button) else 0

    def _pressed_now(self, button):
        """True only on the tick the button goes down."""
        cur = self._btn(button)
        prev = self._prev_buttons.get(button, 0)
        self._prev_buttons[button] = cur
        return cur and not prev

    def _read_pad(self, dt):
        # Edge-triggered buttons
        if self._pressed_now(pygame.CONTROLLER_BUTTON_START):  # Options
            self.quit()
            return
        if self._pressed_now(pygame.CONTROLLER_BUTTON_A):  # Cross
            self.call_service()
        if self._pressed_now(pygame.CONTROLLER_BUTTON_X):  # Square
            self.z = 0.0
        if self._pressed_now(pygame.CONTROLLER_BUTTON_Y):  # Triangle
            self.q = (0.0, 0.0, 0.0, 1.0)
        if self._pressed_now(pygame.CONTROLLER_BUTTON_B):  # Circle
            self.x = self.y = 0.0

        self.fine = bool(self._btn(pygame.CONTROLLER_BUTTON_LEFTSTICK))  # L3
        k = self.fine_scale if self.fine else 1.0

        # Translation (left stick: right = +x, up = +y)
        lx, ly = radial_deadzone(
            self._axis(pygame.CONTROLLER_AXIS_LEFTX),
            -self._axis(pygame.CONTROLLER_AXIS_LEFTY),
            self.deadzone,
        )
        self.x = clamp(
            self.x + lx * self.lin_speed * k * dt, -self.xy_limit, self.xy_limit
        )
        self.y = clamp(
            self.y + ly * self.lin_speed * k * dt, -self.xy_limit, self.xy_limit
        )

        dz = self._btn(pygame.CONTROLLER_BUTTON_DPAD_UP) - self._btn(
            pygame.CONTROLLER_BUTTON_DPAD_DOWN
        )
        self.z = clamp(self.z + dz * self.z_speed * k * dt, -self.z_limit, self.z_limit)

        # Tilt (right stick), same convention as the mouse drag
        rx, ry = radial_deadzone(
            self._axis(pygame.CONTROLLER_AXIS_RIGHTX),
            -self._axis(pygame.CONTROLLER_AXIS_RIGHTY),
            self.deadzone,
        )
        mag = math.hypot(rx, ry)
        if mag > 1e-9:
            q_tilt = axis_angle_quat(
                rx / mag, ry / mag, 0.0, mag * self.rot_speed * k * dt
            )
            self.q = quat_mul(q_tilt, self.q)

        # Spin about the tool's own Z (L1 / R1)
        spin = self._btn(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER) - self._btn(
            pygame.CONTROLLER_BUTTON_LEFTSHOULDER
        )
        if spin:
            q_spin = axis_angle_quat(0.0, 0.0, 1.0, spin * self.spin_speed * k * dt)
            self.q = quat_mul(self.q, q_spin)
        self.q = quat_normalize(self.q)

        # Gripper (R2 = close, L2 = open; the deeper trigger wins)
        r2 = self._trigger(pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
        l2 = self._trigger(pygame.CONTROLLER_AXIS_TRIGGERLEFT)
        action, depth = None, 0.0
        if r2 > self.trig_thr or l2 > self.trig_thr:
            action, depth = ("close", r2) if r2 >= l2 else ("open", l2)
        self._update_gripper(action, depth)

    # ---------------- service / sliders ----------------
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
            self.get_logger().warning(
                f"Service disabled: cannot load {self.service_type!r} "
                f"for {self.service_name!r}: {e}"
            )
            return None

    def call_service(self):
        client = self.service_client
        if client is None:
            return
        if not client.service_is_ready():
            self.get_logger().warning(
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
            self.get_logger().warning(f"Service {self.service_name} failed: {e}")

    def _slider_window(self):
        """Sensitivity sliders in a separate Tk window (own thread)."""
        try:
            root = tk.Tk()
            root.title("PS4 sensitivity")
            self._slider_root = root
            for lo, hi, res, label, attr in (
                (0.01, 0.5, 0.01, "X/Y speed (m/s)", "lin_speed"),
                (0.005, 0.25, 0.005, "Z speed (m/s)", "z_speed"),
                (0.1, 2.0, 0.05, "Tilt speed (rad/s)", "rot_speed"),
                (0.1, 3.0, 0.05, "Spin speed (rad/s)", "spin_speed"),
                (0, 255, 1, "Gripper max speed (0-255)", None),
            ):
                value = self.fast_spe if attr is None else getattr(self, attr)
                scale = tk.Scale(
                    root,
                    from_=lo,
                    to=hi,
                    resolution=res,
                    orient="horizontal",
                    label=label,
                    length=360,
                    command=lambda v, a=attr: self._set_sensitivity(a, v),
                )
                scale.set(value)
                scale.pack(fill="x")
            root.mainloop()
        except Exception as e:
            self.get_logger().warning(f"Sensitivity sliders disabled: {e}")

    def _set_sensitivity(self, attr, value):
        if attr is None:
            self.fast_spe = clamp255(float(value))
        else:
            setattr(self, attr, float(value))

    # ---------------- gripper ----------------
    def _dir(self, action):
        """+1 if this action moves towards 255, -1 if towards 0."""
        close_sign = 1 if self.grip_closed >= self.grip_open else -1
        return close_sign if action == "close" else -close_sign

    def _start_motion(self, spe):
        """Send ONE move to the end position; the gripper moves smoothly by itself."""
        target = self.grip_closed if self.grip_action == "close" else self.grip_open
        # The firmware only applies a new speed together with a new target,
        # so alternate the target by one count every time we resend.
        self._nudge = not self._nudge
        if self._nudge:
            target -= self._dir(self.grip_action)
        self.grip_spe = spe
        self.gripper.move(target, spe)
        self._last_spe_send = time.monotonic()

    def _update_gripper(self, action, depth):
        if action is None:
            if self.grip_action is not None:
                self.gripper.stop_here(self._dir(self.grip_action))
                self.grip_action = None
            return

        frac = clamp((depth - self.trig_thr) / (1.0 - self.trig_thr), 0.0, 1.0)
        spe = clamp255(self.slow_spe + frac * (self.fast_spe - self.slow_spe))

        if action != self.grip_action:
            self.grip_action = action
            self._start_motion(spe)
        elif (
            abs(spe - self.grip_spe) >= 8
            and time.monotonic() - self._last_spe_send >= 0.1
        ):
            self._start_motion(spe)

    def _grip_bar(self, p, width=20):
        span = self.grip_closed - self.grip_open
        frac = 0.0 if span == 0 else (p - self.grip_open) / span
        frac = clamp(frac, 0.0, 1.0)
        n = int(round(frac * width))
        return "[" + "#" * n + "-" * (width - n) + f"] {frac * 100:5.1f}% closed"

    # ---------------- main tick ----------------
    def tick(self):
        if not self.running:
            return
        now = time.monotonic()
        dt = min(0.1, now - self._last_tick)
        self._last_tick = now

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.quit()
                return
            if event.type == pygame.KEYDOWN and event.key in (
                pygame.K_ESCAPE,
                pygame.K_q,
            ):
                self.quit()
                return

        if self._ensure_pad():
            self._read_pad(dt)
            if not self.running:
                return

        self.publish_pose()

        self._frame += 1
        if self._frame % self.draw_every == 0:
            self._draw()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # Same axis mapping as the mouse version
        msg.pose.position.x = -self.y
        msg.pose.position.y = self.x
        msg.pose.position.z = self.z
        qx, qy, qz, qw = self.q
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)

    # ---------------- status window ----------------
    def _draw(self):
        qx, qy, qz, qw = self.q
        zx = 2 * (qx * qz + qw * qy)
        zy = 2 * (qy * qz - qw * qx)
        zz = 1 - 2 * (qx * qx + qy * qy)
        tilt = math.degrees(math.acos(clamp(zz, -1.0, 1.0)))
        direction = math.degrees(math.atan2(zy, zx)) if tilt > 1e-6 else 0.0

        if self.grip_action:
            grip_state = "CLOSING" if self.grip_action == "close" else "OPENING"
            grip_state += f"  speed {self.grip_spe:3d}"
        elif self.gripper.holding:
            grip_state = "HOLDING object"
        else:
            grip_state = "idle"
        actual = self.gripper.actual
        actual_txt = (
            "n/a" if actual is None else f"{actual:3d} {self._grip_bar(actual)}"
        )
        pad_txt = self.pad_name if self.pad is not None else "NOT CONNECTED"

        lines = [
            f"controller : {pad_txt}",
            f"mode       : {'PRECISION' if self.fine else 'normal'}",
            f"x          : {self.x:+.4f} m",
            f"y          : {self.y:+.4f} m",
            f"z          : {self.z:+.4f} m",
            f"tilt       : {tilt:7.2f} deg  towards {direction:+7.1f} deg",
            f"quat       : x={qx:+.3f} y={qy:+.3f} z={qz:+.3f} w={qw:+.3f}",
            f"gripper    : {grip_state}   socket: {self.gripper.status}",
            f"  pos      : {actual_txt}",
            "",
            "L-stick = X/Y    D-pad up/down = Z    hold L3 = precision",
            "R-stick = tilt   L1/R1 = spin",
            "R2 = close gripper   L2 = open (deeper = faster)",
            "Cross = call service   Square = reset Z",
            "Triangle = reset orientation   Circle = reset X/Y",
            "Options = quit",
        ]
        self.screen.fill((30, 30, 30))
        for i, line in enumerate(lines):
            colour = (
                (255, 120, 100)
                if line.startswith("controller : NOT")
                else (220, 220, 220)
            )
            self.screen.blit(self.font.render(line, True, colour), (10, 10 + i * 22))
        pygame.display.flip()

    # ---------------- shutdown ----------------
    def quit(self):
        if not self.running:
            return
        self.running = False
        if self.grip_action is not None:
            self.gripper.stop_here(self._dir(self.grip_action))
            self.grip_action = None
            time.sleep(0.15)  # let the worker send the stop before shutting down
        self.gripper.shutdown()


def main():
    rclpy.init()
    node = PS4PosePublisher()
    try:
        while node.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        node.quit()
    finally:
        node.gripper.shutdown()
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
