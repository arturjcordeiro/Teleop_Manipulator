#pragma once

#include "teleop_manipulators_core/manipulator_interface.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <mutex>
#include <string>
#include <thread>

namespace teleop {

class UrSimRobot : public ManipulatorInterface {
public:
  UrSimRobot() = default;
  ~UrSimRobot() override;

  bool init(const RobotContext &ctx) override;
  bool connect() override;
  void disconnect() noexcept override;
  std::optional<JointState> read() override;
  bool write(const JointCommand &cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override {
    return mode_;
  }
  [[nodiscard]] double period() const noexcept override { return period_; }
  [[nodiscard]] std::size_t dof() const noexcept override { return 6; }
  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  bool ready();
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish(const Eigen::VectorXd &v);

  double period_{0.002};
  CommandMode mode_{CommandMode::Position};
  double max_joint_step_{0.1};
  double acceleration_{1.0};
  std::string last_error_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  std::mutex mtx_;
  Eigen::VectorXd q_{Eigen::VectorXd::Zero(6)};
  Eigen::VectorXd qd_{Eigen::VectorXd::Zero(6)};
  Clock::time_point last_msg_{};
  bool have_state_ = false;

  Eigen::VectorXd last_vel_cmd_{Eigen::VectorXd::Zero(6)}; // for accel limiting
};

} // namespace teleop
