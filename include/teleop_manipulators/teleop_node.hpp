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

#include <algorithm>
#include <cmath>
#include <std_msgs/msg/string.hpp>

#include "rclcpp_components/register_node_macro.hpp"

#include "teleop_manipulators/backends/robot_factory.hpp"
#include "teleop_manipulators/core/diff_ik.hpp"
#include "teleop_manipulators/core/pose_mapper.hpp"
#include "teleop_manipulators/core/safety.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace teleop {

/**
 * Realtime teleoperation ode.
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

  ~TeleopManipulator() override;

private:
  /// Store the latest received tool pose. Cheap, no computation.
  void ToolPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// Read the latest pose, compute the delta and publish the command.
  void CalculateDelta(geometry_msgs::msg::PoseStamped &pose);
  void PublishManipulatorCommand();
  void onRobotDescription(const std_msgs::msg::String::ConstSharedPtr &msg);

  /// Publish a zero command (used on the deadman timeout).
  void PublishZeroCommand();

  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void
  clutchCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  [[nodiscard]] std::optional<StampedPose> latestPose() const;

  // Control thread: runs at robot_->period().
  void controlLoop();
  void step();
  void disengage();

  // Owned by the control thread only
  Eigen::VectorXd q_cmd_;
  bool engaged_{false};

  DiffIK::Params ik_params_;
  Safety::Params safety_params_;
  RobotConfig robot_config_;
  std::string pose_topic_;

  // Core + backend (created in onRobotDescription)
  std::unique_ptr<DiffIK> ik_;
  std::unique_ptr<Safety> safety_;
  std::unique_ptr<PoseMapper> mapper_;
  std::unique_ptr<RobotInterface> robot_;

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clutch_srv_;
  std::thread control_thread_;
  rclcpp::Publisher<ibot_common::msg::ArmCommand>::SharedPtr
      robot_command_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Pre-allocated outgoing message, reused on every tick
  ibot_common::msg::ArmCommand command_msg_;

  mutable std::mutex pose_mutex_;
  std::optional<StampedPose> latest_pose_;
  std::atomic<bool> clutch_request_{false};
  std::atomic<bool> running_{true};

  // Shared state between the pose callback and the control timer
  // std::mutex pose_mutex_;
  // geometry_msgs::msg::PoseStamped latest_pose_;
  // rclcpp::Time latest_pose_stamp_;
  // bool has_pose_{false};

  // Previous pose used to compute the delta
  geometry_msgs::msg::Pose previous_pose_;
  bool has_previous_pose_{false};
};

} // namespace teleop

#endif // TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
