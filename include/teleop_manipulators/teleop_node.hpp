#ifndef TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
#define TELEOP_MANIPULATORS__TELEOP_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ibot_common/msg/arm_command.hpp>

namespace teleop {

/**
 * Realtime teleoperation node.
 *
 * Receives the tool pose from the teleoperation device, computes the delta with
 * respect to the previous pose and publishes it as a cartesian command for the
 * robot.
 *
 * The pose input and the command output are decoupled: the pose callback only
 * stores the latest sample, while a fixed-rate timer performs the delta
 * computation at the control rate. This keeps the outgoing command stream
 * smooth regardless of the jitter of the teleoperation device.
 */
class TeleopManipulator : public rclcpp::Node {
public:
  explicit TeleopManipulator(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~TeleopManipulator() override = default;

private:
  /// Store the latest received tool pose. Cheap, no computation.
  void ToolPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// Read the latest pose, compute the delta and publish the command.
  void CalculateDelta(geometry_msgs::msg::PoseStamped &pose);
  void PublishManipulatorCommand();

  /// Publish a zero command (used on the deadman timeout).
  void PublishZeroCommand();

  // Parameters
  std::string tool_pose_topic_;
  std::string command_topic_;
  double control_rate_hz_{0.0};
  double translation_scale_{0.0};
  double rotation_scale_{0.0};
  double max_translation_step_{0.0};
  double max_rotation_step_{0.0};
  double deadband_{0.0};
  double pose_timeout_s_{0.0};

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      tool_pose_sub_;
  rclcpp::Publisher<ibot_common::msg::ArmCommand>::SharedPtr
      robot_command_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Pre-allocated outgoing message, reused on every tick
  ibot_common::msg::ArmCommand command_msg_;

  // Shared state between the pose callback and the control timer
  std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  rclcpp::Time latest_pose_stamp_;
  bool has_pose_{false};

  // Previous pose used to compute the delta
  geometry_msgs::msg::Pose previous_pose_;
  bool has_previous_pose_{false};
};

} // namespace teleop

#endif // TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
