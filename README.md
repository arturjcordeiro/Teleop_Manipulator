# teleop_manipulators

ROS 2 package for teleoperating one or more manipulators with an OptiTrack
motion-capture system.

The node receives a tracked tool pose, maps it relative to the pose captured at
clutch time, solves differential IK against the robot URDF, applies safety
limits, and streams joint commands to a robot backend. Supported backends:

| Backend | `robot.type` | Notes |
| --- | --- | --- |
| Mock | `mock` | Integrates commands; no hardware, no ROS controller |
| UR simulation | `ur_sim` | Uses `ros2_control` `forward_position_controller` / `forward_velocity_controller` |
| Universal Robots | `ur` | Real robot through `ur_rtde`; built only if `ur_rtde` is found |
| KUKA iiwa 14 | `kuka` | `robot.kuka_interface: udp` (SmartServoPTP) or `fri` (KUKA FRI) |

## How it works

```
OptiTrack tool pose ──> PoseMapper ──> DiffIK ──> Safety ──> robot backend
 /optitrack/tool/pose   (clutch +     (Pinocchio)  (limits,    (mock / UR /
                         relative                   timeout)    KUKA)
                         mapping)
```

- The pose callback only stores the latest sample. A separate control thread runs
  at the robot period.
- `PoseMapper` anchors the tracked tool pose to the current TCP pose on the
  clutch rising edge. Tool motion is then rotated into the robot base frame,
  scaled, and applied to that anchor.
- `DiffIK` solves damped-least-squares IK from the robot URDF.
- `Safety` enforces joint velocity limits, position limits, and a pose timeout.
- Missing or stale OptiTrack data, a lost robot connection, or clutch release
  disengages teleoperation and stops the robot.

## Requirements

- ROS 2 (tested on Jazzy)
- Eigen3
- Pinocchio
- KUKA FRI client SDK (`fri_client_sdk`, imported via `fri.repos`)
- `ur_rtde` (optional, for real UR robots)
- For UR simulation: `ur_robot_driver` and `ur_description`
- For KUKA: a cabinet running `SmartServoPTP.java` (UDP) or FRI with a matching
  FRI client version (see `fri.repos`)

```bash
# Pinocchio
sudo apt install ros-$ROS_DISTRO-pinocchio

# Optional: real Universal Robots support
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt update
sudo apt install librtde librtde-dev
```

## Build

```bash
cd ~/dev_ws
vcs import src < src/Teleop_Manipulator/fri.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to teleop_manipulators
source install/setup.bash
```

`fri.repos` pins FRI 1.15. Set its `version` to the branch matching your
controller before importing (`fri-1.11`, `fri-1.13`, `fri-1.14`, `fri-1.15`,
`fri-1.16`, `fri-2.5`, `fri-2.6`, `fri-2.7`). A mismatch fails the FRI build.

## Run

The node needs a `robot_description` (URDF) before it starts. With
`robot_state_publisher` or a robot driver, publish it on the configured
`robot_description_topic` as a transient-local `std_msgs/String`.

```bash
ros2 launch teleop_manipulators run.launch.py robot:=ur_sim
```

`robot:=<name>` selects `config/<name>.yaml`. Available configs: `mock`,
`ur_sim`, `ur5`, `ur10`, `kuka`.

Before the node is ready it logs `Waiting for robot description ...`. After the
URDF arrives and the backend connects it logs `Teleop ready (... DOF)`.

### Engage / release the clutch

Teleoperation is disabled until the clutch is engaged:

```bash
ros2 service call /teleop_manipulators_node/clutch \
  std_srvs/srv/SetBool "{data: true}"

ros2 service call /teleop_manipulators_node/clutch \
  std_srvs/srv/SetBool "{data: false}"
```

Engaging anchors the current tracked tool pose to the current robot TCP pose.
Release, then re-engage, to reposition the tool without moving the robot.

### Input pose

Publish `geometry_msgs/PoseStamped` on `pose_topic`
(default `/optitrack/tool/pose`). The pose is expected in the OptiTrack world
frame. The node does not look up TF; it uses the local receive time for the
pose-timeout check, so a clock difference between OptiTrack and the robot
computer does not matter.

`mapping.R_base_optitrack` (row-major 3x3) rotates OptiTrack coordinates into
the robot base frame. The default identity assumes Z-up tracking data. For
Motive's default Y-up stream use:

```yaml
mapping:
  R_base_optitrack: [1.0, 0.0, 0.0,
                     0.0, 0.0, -1.0,
                     0.0, 1.0, 0.0]
```

## Configuration

All parameters live in `config/*.yaml` under `/**/ros__parameters`.

| Parameter | Description |
| --- | --- |
| `pose_topic` | OptiTrack tool pose topic (`geometry_msgs/PoseStamped`) |
| `robot_description_topic` | URDF topic (`std_msgs/String`, transient-local) |
| `publish_joint_states` | Publish measured joints on `/joint_states` (default `false`). Enable it for backends without a ROS driver (KUKA, mock); leave it off for UR where the driver already publishes |
| `joint_state_publish_rate` | Maximum `/joint_states` rate in Hz (default `50`); `0` publishes every control cycle |
| `tcp_frame` | URDF frame used as the controlled TCP, e.g. `tool0` |
| `base_frame` | Manipulator base frame; empty keeps the full chain to the TCP and locks side branches |
| `robot.type` | `mock`, `ur_sim`, `ur`, or `kuka` |
| `robot.ip`, `robot.port` | Robot address (SmartServo UDP address for KUKA) |
| `robot.kuka_interface` | `udp` (SmartServoPTP) or `fri` (KUKA FRI) |
| `robot.fri_ip`, `robot.fri_port` | KONI address when `robot.kuka_interface: fri` |
| `robot.period` | Control period in seconds, e.g. `0.002` for 500 Hz |
| `robot.mode` | `position` or `velocity` |
| `robot.max_joint_step` | Reject position targets further than this from actual q [rad]. Raise for PTP tracking lag; too high weakens the discontinuity guard |
| `ik.damping` | DLS damping; higher is safer near singularities |
| `ik.gain_linear`, `ik.gain_angular` | P-gains on pose error [1/s] |
| `mapping.R_base_optitrack` | 9 row-major values: OptiTrack -> robot base |
| `mapping.translation_scale` | Scale applied to tracked translation |
| `safety.max_joint_velocity` | Per-joint velocity limits [rad/s] |
| `safety.limit_margin` | Keep this far from URDF joint limits [rad] |
| `safety.pose_timeout_ms` | Stop if no fresh pose within this time |

If `safety.max_joint_velocity` does not match the URDF joint count, the URDF
velocity limits are used and a warning is logged.

UR-specific: `robot.servo_lookahead`, `robot.servo_gain`, `robot.acceleration`.
KUKA-specific: `robot.kuka_interface`, `robot.fri_ip`, `robot.fri_port`,
`robot.kuka_command` (`joint` or `cartesian`),
`robot.kuka_speed_rel`, `robot.kuka_accel_rel`, `robot.kuka_speed_mm_s`,
`robot.kuka_accel_mm_s2`.

For KUKA there is no ROS driver, so `config/kuka.yaml` sets
`publish_joint_states: true`: the teleop node republishes the state it already
reads from the UDP/FRI backend on `/joint_states`. Run
`robot_state_publisher` with the KUKA URDF as well, otherwise there is no TF
for RViz. Do not enable this for UR — `ur_robot_driver` already publishes
`/joint_states`.

## Examples

### UR mock hardware

```bash
# Terminal 1: UR driver with mock hardware
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10 robot_ip:=0.0.0.0 \
  use_mock_hardware:=true launch_rviz:=true \
  initial_joint_controller:=forward_position_controller

# Terminal 2: teleop node
ros2 launch teleop_manipulators run.launch.py robot:=ur_sim
```

`ur_sim` reads `/joint_states` and publishes
`/forward_position_controller/commands` (or
`/forward_velocity_controller/commands` in velocity mode).

## Tests

```bash
colcon test --packages-select teleop_manipulators
colcon test-result --verbose
```

Unit tests cover the pose mapper, differential IK, and KUKA wire protocol.
