#include "teleop_manipulators/teleop_node.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace teleop {

namespace {
constexpr double kMinControlRateHz = 1.0;
constexpr double kDefaultControlRateHz = 100.0;

/// Clamp a value into [-limit, +limit]. A non-positive limit disables clamping.
double ClampStep(double value, double limit) {
  if (limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -limit, limit);
}
} // namespace

TeleopManipulator::TeleopManipulator(const rclcpp::NodeOptions &options)
    : rclcpp::Node("teleop_manipulator_node", options) {
  // ---------------------------------------------------------------------------
  // Parameters. Each one is declared explicitly so that a typo in a YAML file
  // or a launch argument fails loudly instead of being silently ignored.
  // ---------------------------------------------------------------------------
  tool_pose_topic_ =
      declare_parameter<std::string>("tool_pose_topic", "/teleop/tool_pose");
  command_topic_ =
      declare_parameter<std::string>("command_topic", "/teleop/cmd_delta");

  control_rate_hz_ =
      declare_parameter<double>("control_rate_hz", kDefaultControlRateHz);

  translation_scale_ = declare_parameter<double>("translation_scale", 1.0);
  rotation_scale_ = declare_parameter<double>("rotation_scale", 1.0);

  // Per-cycle limits. A non-positive value disables the corresponding clamp.
  max_translation_step_ =
      declare_parameter<double>("max_translation_step", 0.02); // [m]
  max_rotation_step_ =
      declare_parameter<double>("max_rotation_step", 0.05); // [rad]

  deadband_ = declare_parameter<double>("deadband", 1e-4);

  pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.2);

  if (control_rate_hz_ < kMinControlRateHz) {
    RCLCPP_WARN(get_logger(),
                "control_rate_hz (%.3f) is too low, falling back to %.1f Hz",
                control_rate_hz_, kDefaultControlRateHz);
    control_rate_hz_ = kDefaultControlRateHz;
  }

  // ---------------------------------------------------------------------------
  // ROS interfaces.
  //
  // The pose subscriber lives in its own mutually exclusive callback group so a
  // slow robot-side callback can never stall pose ingestion.
  // ---------------------------------------------------------------------------
  pose_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                            /*automatically_add_to_executor_with_node=*/false);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = pose_callback_group_;

  // Best effort, depth 1: we always want the newest pose, never a backlog.
  tool_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      tool_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TeleopManipulator::ToolPoseCallback, this,
                std::placeholders::_1),
      sub_options);

  robot_command_publisher_ = create_publisher<ibot_common::msg::ArmCommand>(
      command_topic_, rclcpp::QoS(1));

  // ---------------------------------------------------------------------------
  // Fixed-rate control loop. Publishing from a timer instead of from the pose
  // callback decouples the (jittery) input rate from the (smooth) output rate.
  // ---------------------------------------------------------------------------
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
  control_timer_ = create_wall_timer(
      period, std::bind(&TeleopManipulator::PublishManipulatorCommand, this));

  RCLCPP_INFO(get_logger(),
              "Teleop manipulator ready | pose: %s | command: %s | rate: %.1f "
              "Hz",
              tool_pose_topic_.c_str(), command_topic_.c_str(),
              control_rate_hz_);
}

void TeleopManipulator::ToolPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Hot path: only store the sample. No computation, no logging, no allocation.
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = *msg;
  latest_pose_stamp_ = msg->header.stamp;
  has_pose_ = true;
}

void TeleopManipulator::CalculateDelta(geometry_msgs::msg::PoseStamped &pose) {

  // First valid sample: nothing to differentiate against yet.
  if (!has_previous_pose_) {
    previous_pose_ = pose.pose;
    has_previous_pose_ = true;
    return;
  }

  // --- Translation delta -----------------------------------------------------
  double dx = pose.pose.position.x - previous_pose_.position.x;
  double dy = pose.pose.position.y - previous_pose_.position.y;
  double dz = pose.pose.position.z - previous_pose_.position.z;

  // --- Rotation delta --------------------------------------------------------
  // Relative rotation expressed as a rotation vector. Euler angles are never
  // interpolated here.
  const double q1x = previous_pose_.orientation.x;
  const double q1y = previous_pose_.orientation.y;
  const double q1z = previous_pose_.orientation.z;
  const double q1w = previous_pose_.orientation.w;

  const double q2x = pose.pose.orientation.x;
  const double q2y = pose.pose.orientation.y;
  const double q2z = pose.pose.orientation.z;
  const double q2w = pose.pose.orientation.w;

  // dq = q2 * conjugate(q1)
  const double dqw = q2w * q1w + q2x * q1x + q2y * q1y + q2z * q1z;
  const double dqx = -q2w * q1x + q2x * q1w - q2y * q1z + q2z * q1y;
  const double dqy = -q2w * q1y + q2x * q1z + q2y * q1w - q2z * q1x;
  const double dqz = -q2w * q1z - q2x * q1y + q2y * q1x + q2z * q1w;

  // Rotation vector = 2 * atan2(|v|, w) * v / |v|
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  const double sin_half = std::sqrt(dqx * dqx + dqy * dqy + dqz * dqz);
  if (sin_half > 1e-9) {
    const double angle = 2.0 * std::atan2(sin_half, std::fabs(dqw));
    const double k = (dqw < 0.0 ? -angle : angle) / sin_half;
    rx = dqx * k;
    ry = dqy * k;
    rz = dqz * k;
  }

  // --- Scaling, deadband and clamping ----------------------------------------
  dx *= translation_scale_;
  dy *= translation_scale_;
  dz *= translation_scale_;

  rx *= rotation_scale_;
  ry *= rotation_scale_;
  rz *= rotation_scale_;

  if (deadband_ > 0.0) {
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < deadband_) {
      dx = 0.0;
      dy = 0.0;
      dz = 0.0;
    }
    if (std::sqrt(rx * rx + ry * ry + rz * rz) < deadband_) {
      rx = 0.0;
      ry = 0.0;
      rz = 0.0;
    }
  }

  dx = ClampStep(dx, max_translation_step_);
  dy = ClampStep(dy, max_translation_step_);
  dz = ClampStep(dz, max_translation_step_);

  rx = ClampStep(rx, max_rotation_step_);
  ry = ClampStep(ry, max_rotation_step_);
  rz = ClampStep(rz, max_rotation_step_);
}

void TeleopManipulator::PublishManipulatorCommand() {
  geometry_msgs::msg::PoseStamped pose;
  rclcpp::Time pose_stamp;
  bool has_pose = false;

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose = latest_pose_;
    pose_stamp = latest_pose_stamp_;
    has_pose = has_pose_;
  }

  if (!has_pose) {
    return;
  }

  const rclcpp::Time now = this->now();

  // Deadman: if the teleoperation device stopped sending poses, command zero so
  // the robot does not keep moving on a stale velocity.
  const double pose_age_s =
      (pose_stamp.nanoseconds() == 0) ? 0.0 : (now - pose_stamp).seconds();
  if (pose_timeout_s_ > 0.0 && pose_age_s > pose_timeout_s_) {
    PublishZeroCommand();
    has_previous_pose_ = false;
    return;
  }

  // Calculate Delta
  TeleopManipulator::CalculateDelta(pose);

  // --- Publish ---------------------------------------------------------------
  // The pre-allocated message is reused: no allocation in the control loop.
  // `rclcpp::Time` converts implicitly to `builtin_interfaces::msg::Time`, so
  // the node clock can be assigned straight into the header. Using the node
  // clock (instead of std::chrono / system_clock) keeps the stamp consistent
  // with the rest of the graph and honours `use_sim_time`.
  command_msg_.header.stamp = now;
  command_msg_.header.frame_id = pose.header.frame_id;

  robot_command_publisher_->publish(command_msg_);

  previous_pose_ = pose.pose;
}

void TeleopManipulator::PublishZeroCommand() {
  command_msg_.transforms.clear();

  // Stamp with the node clock at publish time, exactly like the normal command
  // path, so the driver can order/expire commands consistently.
  command_msg_.header.stamp = this->now();
  command_msg_.header.frame_id = "";

  geometry_msgs::msg::TransformStamped transform;
  // Publish all zeros
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  command_msg_.transforms.push_back(transform);
  command_msg_.acceleration = 0.5;
  command_msg_.speed = 0.5;

  robot_command_publisher_->publish(command_msg_);
}

} // namespace teleop

RCLCPP_COMPONENTS_REGISTER_NODE(teleop::TeleopManipulator)
