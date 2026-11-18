# teleop_manipulators

ROS 2 stack for teleoperating one or more manipulators with an OptiTrack
motion-capture system. Everything — the node, the plugin contract, and every
robot backend — lives in this single folder as a monorepo of colcon packages.

The node receives a tracked tool pose, maps it relative to the pose captured at
clutch time, solves differential IK against the robot URDF, applies safety
limits, and streams joint commands to a robot backend.

## Layout

```
Teleop_Manipulator/
  teleop_manipulators_core/   # plugin contract (ManipulatorInterface, types)
  teleop_node/                # the teleop node (package: teleop_manipulators)
  teleop_mock/                # mock backend
  teleop_ur_sim/              # UR ros2_control simulation backend
  teleop_ur_rtde/             # real UR via ur_rtde
  teleop_kuka_udp/            # KUKA SmartServoPTP (UDP)
  teleop_kuka_fri/            # KUKA FRI (KONI)
```

| Package | Role |
| --- | --- |
| `teleop_manipulators_core` | Plugin contract (`ManipulatorInterface`, data types) shared by the node and every backend |
| `teleop_manipulators` (folder `teleop_node/`) | The teleop node + core algorithms (mapping, IK, safety). Knows no concrete robot. |
| `teleop_mock` | `mock` backend — integrates commands, no hardware |
| `teleop_ur_sim` | `ur_sim` backend — `ros2_control` forward position/velocity controllers |
| `teleop_ur_rtde` | `ur` backend — real Universal Robots via `ur_rtde` |
| `teleop_kuka_udp` | `kuka` backend — SmartServoPTP over UDP |
| `teleop_kuka_fri` | `kuka_fri` backend — KUKA FRI (KONI) |

Backends are loaded at runtime through `pluginlib`: the node only depends on
`teleop_manipulators_core` and resolves `robot.type` (the plugin class name) to
a concrete driver. Adding a robot = a new driver package, no changes to the
node (see "Adding a new backend").

| Backend | `robot.type` | Notes |
| --- | --- | --- |
| Mock | `mock` | Integrates commands; no hardware, no ROS controller |
| UR simulation | `ur_sim` | Uses `ros2_control` `forward_position_controller` / `forward_velocity_controller` |
| Universal Robots | `ur` | Real robot through `ur_rtde` |
| KUKA iiwa 14 (UDP) | `kuka` | SmartServoPTP |
| KUKA iiwa 14 (FRI) | `kuka_fri` | KUKA FRI (KONI) |

## How it works

```
OptiTrack tool pose ──> PoseMapper ──> DiffIK ──> Safety ──> robot backend
 /optitrack/tool/pose   (clutch +     (Pinocchio)  (limits,    (pluginlib)
                         relative                   timeout)
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
- Per-driver requirements live in each driver package: `ur_rtde`
  (`teleop_ur_rtde`), KUKA FRI client SDK (`teleop_kuka_fri`, via its
  `fri.repos`), `ur_robot_driver` + `ur_description` (`teleop_ur_sim`).

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
# Optional: KUKA FRI client SDK (only needed for teleop_kuka_fri)
vcs import src < src/Teleop_Manipulator/teleop_kuka_fri/fri.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

`fri.repos` pins FRI 1.15. Set its `version` to the branch matching your
controller before importing (`fri-1.11`, `fri-1.13`, `fri-1.14`, `fri-1.15`,
`fri-1.16`, `fri-2.5`, `fri-2.6`, `fri-2.7`). A mismatch fails the FRI build.

## Run

The node needs a `robot_description` (URDF) before it starts. With
`robot_state_publisher` or a robot driver, publish it on the configured
`robot_description_topic` as a transient-local `std_msgs/String`.

Each driver package ships a launch file that loads its own config:

```bash
ros2 launch teleop_mock mock.launch.py
ros2 launch teleop_ur_sim ur_sim.launch.py
ros2 launch teleop_ur_rtde ur.launch.py robot:=ur5   # or robot:=ur10
ros2 launch teleop_kuka_udp kuka.launch.py
ros2 launch teleop_kuka_fri kuka_fri.launch.py
```

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

Node parameters live in the driver's `config/*.yaml` under
`/**/ros__parameters`. Shared (node-level) parameters:

| Parameter | Description |
| --- | --- |
| `pose_topic` | OptiTrack tool pose topic (`geometry_msgs/PoseStamped`) |
| `robot_description_topic` | URDF topic (`std_msgs/String`, transient-local) |
| `publish_joint_states` | Publish measured joints on `/joint_states` (default `false`). Enable it for backends without a ROS driver (KUKA, mock); leave it off for UR where the driver already publishes |
| `joint_state_publish_rate` | Maximum `/joint_states` rate in Hz (default `50`); `0` publishes every control cycle |
| `tcp_frame` | URDF frame used as the controlled TCP, e.g. `tool0` |
| `base_frame` | Manipulator base frame; empty keeps the full chain to the TCP and locks side branches |
| `robot.type` | Plugin class name: `mock`, `ur_sim`, `ur`, `kuka`, `kuka_fri` |
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

Each backend declares and reads its own `robot.*` parameters (the node does not
know about them). See the driver's `config/*.yaml` for its options, e.g.
UR-specific `robot.servo_lookahead`, `robot.servo_gain`, `robot.acceleration`,
`robot.ip`, `robot.period`, `robot.mode`; KUKA UDP `robot.kuka_command`
(`joint` or `cartesian`), `robot.kuka_speed_rel`, etc.; KUKA FRI `robot.fri_ip`,
`robot.fri_port`.

For KUKA there is no ROS driver, so the config sets `publish_joint_states:
true`: the teleop node republishes the state it already reads from the UDP/FRI
backend on `/joint_states`. Run `robot_state_publisher` with the KUKA URDF as
well, otherwise there is no TF for RViz. Do not enable this for UR —
`ur_robot_driver` already publishes `/joint_states`.

## Adding a new backend

A backend is a `pluginlib` plugin in its own package. The node only depends on
`teleop_manipulators_core`, so a new manipulator is a new package — the node and
the contract stay untouched. `teleop_mock` is the minimal template to copy.

1. **Create the package** under the monorepo root, e.g. `teleop_fanuc/`
   (`package.xml`, `CMakeLists.txt`, `src/`, `include/`).

2. **Implement the interface** — subclass `teleop::ManipulatorInterface`. pluginlib
   needs a default constructor, so config moves to `init()`:

   ```cpp
   // fanuc_robot.hpp
   #include "teleop_manipulators_core/manipulator_interface.hpp"
   namespace teleop {
   class FanucRobot final : public ManipulatorInterface {
   public:
     FanucRobot() = default;
     bool init(const RobotContext &ctx) override;  // read robot.* params here
     bool connect() override;
     void disconnect() noexcept override;
     std::optional<JointState> read() override;
     bool write(const JointCommand &cmd) override;
     void stop() noexcept override;
     CommandMode commandMode() const noexcept override { return mode_; }
     double period() const noexcept override { return period_; }
     std::size_t dof() const noexcept override { return dof_; }
     std::string lastError() const override { return last_error_; }
   private:
     double period_{0.002};
     CommandMode mode_{CommandMode::Position};
     std::size_t dof_{0};
     std::string last_error_;
   };
   }
   ```

   In `init()` read your own params from the host node (`ctx.node`), and store
   `ctx.dof` (and `ctx.forward_kinematics` if you command Cartesian space):

   ```cpp
   bool FanucRobot::init(const RobotContext &ctx) {
     period_ = ctx.node.declare_parameter<double>("robot.period", 0.002);
     mode_ = ctx.node.declare_parameter<std::string>("robot.mode", "position") ==
                     "velocity" ? CommandMode::Velocity : CommandMode::Position;
     dof_ = ctx.dof;
     return true;
   }
   ```

   Register the factory at the bottom of the `.cpp`:

   ```cpp
   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(teleop::FanucRobot, teleop::ManipulatorInterface)
   ```

3. **Register the plugin** with a `plugins.xml` (the `<class name>` is what goes
   in `robot.type`):

   ```xml
   <library path="teleop_fanuc">
     <class name="fanuc" type="teleop::FanucRobot" base_class_type="teleop::ManipulatorInterface">
       <description>Fanuc ...</description>
     </class>
   </library>
   ```

   and in `CMakeLists.txt`:

   ```cmake
   find_package(pluginlib REQUIRED)
   find_package(teleop_manipulators_core REQUIRED)
   add_library(teleop_fanuc SHARED src/fanuc_robot.cpp)
   target_link_libraries(teleop_fanuc
     teleop_manipulators_core::teleop_manipulators_core)
   ament_target_dependencies(teleop_fanuc rclcpp pluginlib)
   pluginlib_export_plugin_description_file(teleop_manipulators_core plugins.xml)
   ```

4. **Ship a config + launch file** (`config/fanuc.yaml`, `launch/fanuc.launch.py`)
   so users run `ros2 launch teleop_fanuc fanuc.launch.py`. The launch file
   points at the generic node launcher:

   ```python
   # fanuc.launch.py
   IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           os.path.join(get_package_share_directory("teleop_manipulators"),
                        "launch", "teleop_node.launch.py")),
       launch_arguments={"robot_config_file": config}.items())
   ```

5. The node picks it up with no code changes: build/install `teleop_fanuc`, set
   `robot.type: fanuc` in its config, and launch.

## Examples

### UR mock hardware

```bash
# Terminal 1: UR driver with mock hardware
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10 robot_ip:=0.0.0.0 \
  use_mock_hardware:=true launch_rviz:=true \
  initial_joint_controller:=forward_position_controller

# Terminal 2: teleop node
ros2 launch teleop_ur_sim ur_sim.launch.py
```

`ur_sim` reads `/joint_states` and publishes
`/forward_position_controller/commands` (or
`/forward_velocity_controller/commands` in velocity mode).

## Tests

```bash
colcon test --packages-select teleop_manipulators teleop_kuka_udp teleop_mock
colcon test-result --verbose
```

Unit tests cover the pose mapper, differential IK, the KUKA UDP wire protocol
(`teleop_kuka_udp`), and plugin loading (`teleop_mock`).
