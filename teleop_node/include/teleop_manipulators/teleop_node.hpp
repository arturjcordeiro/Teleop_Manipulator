#ifndef TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
#define TELEOP_MANIPULATORS__TELEOP_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/publisher.hpp>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "teleop_manipulators/core/diff_ik.hpp"
#include "teleop_manipulators/core/pose_mapper.hpp"
#include "teleop_manipulators/core/safety.hpp"
#include "teleop_manipulators_core/manipulator_interface.hpp"
#include <chrono>

namespace teleop {

/**
 * Realtime teleoperation node.
 *
 * Receives the tool pose from the teleoperation device, maps it relative to the
 * pose captured at engage time, and drives the robot TCP toward that target
 * with differential IK.
 *
 * The pose input and the command output are decoupled: the pose callback only
 * stores the latest sample, while a fixed-rate control thread performs the
 * mapping/IK at the robot period.
 */
class TeleopManipulator : public rclcpp::Node {
public:
  explicit TeleopManipulator(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  ~TeleopManipulator() override;

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void onRobotDescription(const std_msgs::msg::String::ConstSharedPtr &msg);
  void
  clutchCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  [[nodiscard]] std::optional<StampedPose> latestPose() const;

  // Control thread: runs at robot_->period().
  void controlLoop();
  void step();
  void disengage();
  void publishJointState(const JointState &state);

  // Owned by the control thread only
  Eigen::VectorXd q_cmd_;
  bool engaged_{false};

  DiffIK::Params ik_params_;
  Safety::Params safety_params_;
  std::string robot_type_{"mock"};
  double max_joint_step_{0.1};
  std::string pose_topic_;
  bool publish_joint_states_{false};
  double joint_state_rate_{50.0};
  std::vector<std::string> joint_names_;
  Clock::time_point last_joint_pub_{};

  // Core + backend (created in onRobotDescription)
  std::unique_ptr<DiffIK> ik_;
  std::unique_ptr<Safety> safety_;
  std::unique_ptr<PoseMapper> mapper_;
  // robot_ must outlive robot_loader_ (reverse declaration order).
  pluginlib::ClassLoader<ManipulatorInterface> robot_loader_;
  std::shared_ptr<ManipulatorInterface> robot_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      smart_servo_joint_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clutch_srv_;
  std::thread control_thread_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  mutable std::mutex pose_mutex_;
  std::optional<StampedPose> latest_pose_;
  std::atomic<bool> clutch_request_{false};
  std::atomic<bool> running_{true};
};

} // namespace teleop

#endif // TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
