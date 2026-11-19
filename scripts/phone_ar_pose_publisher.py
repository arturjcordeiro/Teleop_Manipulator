#!/usr/bin/env python3
"""
Android phone (AR tracking) -> /probe/pose (geometry_msgs/msg/PoseStamped)

Move the phone in space and the probe follows. Position comes from ARCore
(camera + IMU), which Chrome on Android exposes to web pages through WebXR.
The IMU alone can't give position: it drifts within seconds.

On the phone:
  "Hold to move" -> while held, the probe follows the phone's movement and
                    rotation. Release to stop: the probe stays where it is, so
                    you can reposition your hand and hold again (like lifting
                    a mouse off the desk).
  "Rotation on/off" -> turn off to move position only.
  "Reset pose"   -> probe back to position 0 and no rotation.

Axes (when you start AR, the direction the phone's camera faces is "forward"):
  phone right/left       -> robot +x / -x
  phone forward/back     -> robot +y / -y
  phone up/down          -> robot +z / -z

Requirements on the phone:
  - Chrome for Android
  - "Google Play Services for AR" (Play Store) on an ARCore-supported phone
  - A secure page. Easiest is USB (see below), which also avoids Wi-Fi problems.

Connect over USB (most reliable):
  sudo apt install adb             # once
  enable Developer options -> USB debugging on the phone, plug in the cable
  adb reverse tcp:8080 tcp:8080
  then open  http://localhost:8080  in Chrome on the phone

Connect over Wi-Fi:
  open  https://<pc-ip>:8443  and accept the certificate warning.
  If AR still refuses to start, open chrome://flags on the phone, search
  "Insecure origins treated as secure", add  http://<pc-ip>:8080 , relaunch,
  and use that address instead.

Run:
  source /opt/ros/<distro>/setup.bash
  python3 phone_ar_pose_publisher.py

Optional parameters:
  python3 phone_ar_pose_publisher.py --ros-args \
      -p topic:=/probe/pose -p frame_id:=world \
      -p pos_scale:=1.0 -p rot_gain:=1.0 -p jump_limit:=0.05 \
      -p http_port:=8080 -p https_port:=8443 -p rate_hz:=50.0

  pos_scale : 1.0 = probe moves as far as the phone, 0.2 = one fifth (fine work)
  rot_gain  : 1.0 = probe rotates as much as the phone
  jump_limit: if the tracked phone position jumps more than this (metres)
              between two updates, it's treated as a tracking glitch and the
              probe stops until you press "Hold to move" again.
  https_port:=0 disables https.

To change which robot axis each phone movement maps to, edit
xr_to_robot_vec() and xr_to_robot_quat() below (keep them consistent).
"""

import json
import math
import os
import socket
import ssl
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# ---------------------------------------------------------------- frames
# WebXR "local" space: x = right, y = up, -z = forward (where the camera
# pointed at the start). Robot: x = right, y = forward, z = up.
def xr_to_robot_vec(v):
    x, y, z = v
    return (-z, -x, y)


def xr_to_robot_quat(q):
    # Same basis change as above (it is a proper rotation, so the
    # quaternion's vector part transforms like a vector).
    x, y, z, w = q
    return (-z, -x, y, w)


# ---------------------------------------------------------------- quaternions
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def quat_scale_angle(q, gain):
    x, y, z, w = quat_normalize(q)
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    angle = 2.0 * math.acos(min(1.0, w)) * gain
    h = math.sin(angle / 2)
    return (x / s * h, y / s * h, z / s * h, math.cos(angle / 2))


IDENTITY = (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------- web page
PAGE = r"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>Probe AR control</title>
<style>
  :root{--bg:#1d2730; --panel:rgba(29,39,48,.82); --line:#45596a; --text:#e6edf2;
        --muted:#a7b7c3; --go:#5cc8e8; --hold:#f2b134; --bad:#ff6b6b}
  *{box-sizing:border-box; -webkit-tap-highlight-color:transparent; user-select:none; -webkit-user-select:none}
  html,body{margin:0; height:100%; background:var(--bg); color:var(--text);
    font:15px/1.4 system-ui,-apple-system,Roboto,sans-serif}
  #start{min-height:100%; display:flex; flex-direction:column; justify-content:center; gap:14px; padding:32px 24px}
  #start h1{font-size:22px; margin:0}
  #start p{margin:0; color:var(--muted); max-width:36ch}
  #start button{align-self:flex-start; padding:16px 36px; font-size:17px; border-color:var(--go); color:var(--go)}
  #msg{color:var(--hold)}
  #checks{font-size:13px; color:var(--muted)}
  button{font:inherit; color:var(--text); background:var(--panel); border:1px solid var(--line);
    border-radius:14px; padding:14px 8px; touch-action:none}
  #overlay{display:none; position:fixed; inset:0; flex-direction:column; justify-content:space-between;
    padding:calc(env(safe-area-inset-top) + 10px) 12px calc(env(safe-area-inset-bottom) + 14px)}
  #overlay.on{display:flex}
  #top{background:var(--panel); border-radius:14px; padding:10px 12px; font-size:13px}
  #state{font-weight:600; font-size:15px}
  #nums{font-variant-numeric:tabular-nums; color:var(--muted); margin-top:4px}
  #bottom{display:grid; grid-template-columns:1fr 1fr 1fr; gap:8px}
  #hold{grid-column:1 / -1; padding:30px; font-size:19px; font-weight:600;
    border:2px solid var(--hold); color:var(--hold)}
  #hold.on{background:var(--hold); color:var(--bg)}
  .bad{color:var(--bad)}
</style></head>
<body>
<div id="start">
  <h1>Probe AR control</h1>
  <p>Hold the button and move the phone. The probe copies the movement.</p>
  <p>Point the camera at a textured, well-lit surface; plain white walls track badly.</p>
  <p id="msg"></p>
  <button id="go">Start AR</button>
  <p id="checks"></p>
</div>

<div id="overlay">
  <div id="top">
    <div id="state">Starting…</div>
    <div id="nums">x 0 &nbsp; y 0 &nbsp; z 0 &nbsp; tilt 0&deg;</div>
  </div>
  <div id="bottom">
    <button id="hold">Hold to move</button>
    <button id="rot">Rotation on</button>
    <button id="reset">Reset pose</button>
    <button id="exit">Exit</button>
  </div>
</div>

<script>
"use strict";
const $=id=>document.getElementById(id);
let session=null, refSpace=null, gl=null, held=false, rotOn=true, cmds=[], inflight=false, connected=false;

$("checks").textContent =
  "Secure page: "+(window.isSecureContext?"yes":"no — use http://localhost via USB, or https")+
  " · WebXR: "+(navigator.xr?"yes":"no");

function say(t){ $("msg").textContent=t; }

$("go").addEventListener("click", async ()=>{
  if(!window.isSecureContext){ say("AR needs a secure page. Use USB with http://localhost:8080, or the https address."); return; }
  if(!navigator.xr){ say("This browser has no WebXR. Use Chrome for Android."); return; }
  let ok=false;
  try{ ok=await navigator.xr.isSessionSupported("immersive-ar"); }catch(e){}
  if(!ok){ say("AR isn't available. Install or update “Google Play Services for AR” from the Play Store, then reload."); return; }
  try{
    session=await navigator.xr.requestSession("immersive-ar",{
      requiredFeatures:["local"], optionalFeatures:["dom-overlay"], domOverlay:{root:$("overlay")}});
  }catch(e){ say("Couldn't start AR: "+e.message); return; }

  const canvas=document.createElement("canvas");
  gl=canvas.getContext("webgl",{xrCompatible:true});
  session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl)});
  refSpace=await session.requestReferenceSpace("local");
  $("overlay").classList.add("on");
  session.addEventListener("end", ()=>{
    session=null; held=false; $("hold").classList.remove("on");
    $("overlay").classList.remove("on");
    post({tracking:false, clutch:false, rot:rotOn, cmds:[]}, true);
  });
  session.requestAnimationFrame(onFrame);
});

// Touches on our buttons shouldn't also count as AR "select" taps
$("overlay").addEventListener("beforexrselect", e=>e.preventDefault());

const hold=$("hold");
hold.addEventListener("pointerdown", e=>{ hold.setPointerCapture(e.pointerId); held=true; hold.classList.add("on"); hold.textContent="Moving"; });
const release=()=>{ held=false; hold.classList.remove("on"); hold.textContent="Hold to move"; };
hold.addEventListener("pointerup", release); hold.addEventListener("pointercancel", release);
$("rot").addEventListener("click", ()=>{ rotOn=!rotOn; $("rot").textContent=rotOn?"Rotation on":"Rotation off"; });
$("reset").addEventListener("click", ()=>cmds.push("reset_pose"));
$("exit").addEventListener("click", ()=>{ if(session) session.end(); });

function onFrame(t, frame){
  const s=frame.session; s.requestAnimationFrame(onFrame);
  const layer=s.renderState.baseLayer;
  gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
  gl.clearColor(0,0,0,0); gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);   // camera shows through

  const pose=frame.getViewerPose(refSpace);
  const tracking=!!pose && !pose.emulatedPosition;
  let body={tracking, clutch:held, rot:rotOn};
  if(tracking){
    const tr=pose.transform;
    body.p=[tr.position.x, tr.position.y, tr.position.z];
    body.q=[tr.orientation.x, tr.orientation.y, tr.orientation.z, tr.orientation.w];
  }
  post(body, false);
}

function post(body, force){
  if(inflight && !force) return;      // one request at a time; next frame sends fresh data
  inflight=true;
  const sent=cmds; cmds=[]; body.cmds=(body.cmds||[]).concat(sent);
  fetch("/input",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)})
    .then(r=>r.json()).then(st=>{ connected=true; show(st, body.tracking); })
    .catch(()=>{ connected=false; cmds=sent.concat(cmds); show(null, body.tracking); })
    .finally(()=>{ inflight=false; });
}

function show(st, tracking){
  const el=$("state"); el.className="";
  if(!connected){ el.textContent="Not connected to the PC"; el.className="bad"; return; }
  if(!tracking) el.textContent="Looking for surfaces… move the phone slowly";
  else if(st.warning){ el.textContent=st.warning; el.className="bad"; }
  else el.textContent= st.engaged ? "Moving the probe" : "Tracking. Hold the button to move";
  $("nums").innerHTML="x "+st.x.toFixed(3)+" &nbsp; y "+st.y.toFixed(3)+" &nbsp; z "+st.z.toFixed(3)+
                      " &nbsp; tilt "+st.tilt.toFixed(1)+"&deg;";
}
</script></body></html>
"""


# ---------------------------------------------------------------- ROS node
class PhoneARPosePublisher(Node):
    def __init__(self):
        super().__init__("phone_ar_pose_publisher")

        self.declare_parameter("topic", "/probe/pose")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("https_port", 8443)
        self.declare_parameter("pos_scale", 2.0)
        self.declare_parameter("rot_gain", 2.0)
        self.declare_parameter("jump_limit", 0.5)
        self.declare_parameter("rate_hz", 120.0)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.http_port = int(self.get_parameter("http_port").value)
        self.https_port = int(self.get_parameter("https_port").value)
        self.pos_scale = float(self.get_parameter("pos_scale").value)
        self.rot_gain = float(self.get_parameter("rot_gain").value)
        self.jump_limit = float(self.get_parameter("jump_limit").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        self.lock = threading.Lock()
        self.pos = (0.0, 0.0, 0.0)
        self.cur_q = IDENTITY

        self.engaged = False  # probe currently following the phone
        self.rot_active = False
        self.need_repress = False  # set after a tracking glitch
        self.warning = ""
        self.last_input = 0.0

        # anchors captured when "Hold to move" starts
        self.P0 = self.last_P = (0.0, 0.0, 0.0)
        self.Q0 = IDENTITY
        self.B = (0.0, 0.0, 0.0)
        self.BQ = IDENTITY

        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.create_timer(1.0 / rate_hz, self.publish_pose)
        self.get_logger().info(f"Publishing PoseStamped on {topic} at {rate_hz} Hz")

    def handle_input(self, d):
        with self.lock:
            self.last_input = time.monotonic()

            for cmd in d.get("cmds", []):
                if cmd == "reset_pose":
                    self.pos = (0.0, 0.0, 0.0)
                    self.cur_q = IDENTITY
                    self.engaged = False  # re-anchor on the next update

            clutch = bool(d.get("clutch"))
            rot = bool(d.get("rot", True))
            if not clutch:
                self.need_repress = False
                self.warning = ""

            p, q = d.get("p"), d.get("q")
            want = (
                clutch
                and bool(d.get("tracking"))
                and p is not None
                and q is not None
                and not self.need_repress
            )

            if want:
                P = xr_to_robot_vec(tuple(float(c) for c in p))
                Q = quat_normalize(xr_to_robot_quat(tuple(float(c) for c in q)))

                if not self.engaged:
                    self.P0, self.last_P, self.B = P, P, self.pos
                    self.rot_active = False
                    self.engaged = True
                elif math.dist(P, self.last_P) > self.jump_limit:
                    self.engaged = False
                    self.need_repress = True
                    self.warning = "Tracking jumped. Release and hold again."
                    self.get_logger().warn("Tracking jump detected, probe stopped.")
                    return self._state()
                self.last_P = P

                self.pos = tuple(
                    b + (pc - p0) * self.pos_scale
                    for b, pc, p0 in zip(self.B, P, self.P0)
                )

                if rot:
                    if not self.rot_active:  # (re)anchor rotation
                        self.Q0, self.BQ = Q, self.cur_q
                        self.rot_active = True
                    dq = quat_scale_angle(
                        quat_mul(Q, quat_conj(self.Q0)), self.rot_gain
                    )
                    self.cur_q = quat_normalize(quat_mul(dq, self.BQ))
                else:
                    self.rot_active = False
            else:
                self.engaged = False
                self.rot_active = False

            return self._state()

    def _state(self):
        qx, qy, qz, qw = self.cur_q
        zz = 1 - 2 * (qx * qx + qy * qy)
        return {
            "x": self.pos[0],
            "y": self.pos[1],
            "z": self.pos[2],
            "tilt": math.degrees(math.acos(max(-1.0, min(1.0, zz)))),
            "engaged": self.engaged,
            "warning": self.warning,
        }

    def publish_pose(self):
        with self.lock:
            # Phone went quiet (screen off, cable/Wi-Fi drop): stop following
            if self.engaged and time.monotonic() - self.last_input > 0.3:
                self.engaged = False
                self.rot_active = False
                self.get_logger().warn("No data from the phone, probe stopped.")

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.pos
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ) = self.cur_q
        self.pub.publish(msg)


# ---------------------------------------------------------------- web server
def make_handler(node):
    page = PAGE.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send(200, page, "text/html; charset=utf-8")
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path != "/input":
                self._send(404, b"not found", "text/plain")
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(length) or b"{}")
                self._send(
                    200,
                    json.dumps(node.handle_input(data)).encode(),
                    "application/json",
                )
            except Exception as e:
                self._send(400, str(e).encode(), "text/plain")

        def log_message(self, *args):
            pass

    return Handler


def ensure_certificate():
    d = os.path.expanduser("~/.ros/phone_pose_cert")
    cert, key = os.path.join(d, "cert.pem"), os.path.join(d, "key.pem")
    if not (os.path.exists(cert) and os.path.exists(key)):
        os.makedirs(d, exist_ok=True)
        subprocess.run(
            [
                "openssl",
                "req",
                "-x509",
                "-newkey",
                "rsa:2048",
                "-nodes",
                "-keyout",
                key,
                "-out",
                cert,
                "-days",
                "3650",
                "-subj",
                "/CN=ros2-phone-pose",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    return cert, key


def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips) or ["<your-pc-ip>"]


def main():
    rclpy.init()
    node = PhoneARPosePublisher()
    log = node.get_logger()
    servers = []

    http = ThreadingHTTPServer(("0.0.0.0", node.http_port), make_handler(node))
    servers.append(http)

    if node.https_port:
        try:
            cert, key = ensure_certificate()
            https = ThreadingHTTPServer(
                ("0.0.0.0", node.https_port), make_handler(node)
            )
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(cert, key)
            https.socket = ctx.wrap_socket(https.socket, server_side=True)
            servers.append(https)
        except Exception as e:
            log.warn(f"https disabled ({e}). Use USB with http://localhost instead.")

    for s in servers:
        threading.Thread(target=s.serve_forever, daemon=True).start()

    log.info("--- Open on the phone (Chrome) ---")
    log.info(
        f'USB : run "adb reverse tcp:{node.http_port} tcp:{node.http_port}", '
        f"then open http://localhost:{node.http_port}"
    )
    if len(servers) > 1:
        for ip in local_ips():
            log.info(
                f"WiFi: https://{ip}:{node.https_port}  (accept the certificate warning)"
            )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for s in servers:
            s.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
