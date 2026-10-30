#ifndef TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
#define TELEOP_MANIPULATORS__TELEOP_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "teleop_manipulators/backends/robot_factory.hpp"
#include "teleop_manipulators/core/diff_ik.hpp"
#include "teleop_manipulators/core/pose_mapper.hpp"
#include "teleop_manipulators/core/safety.hpp"

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

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clutch_srv_;
  std::thread control_thread_;

  mutable std::mutex pose_mutex_;
  std::optional<StampedPose> latest_pose_;
  std::atomic<bool> clutch_request_{false};
  std::atomic<bool> running_{true};
};

} // namespace teleop

#endif // TELEOP_MANIPULATORS__TELEOP_NODE_HPP_
