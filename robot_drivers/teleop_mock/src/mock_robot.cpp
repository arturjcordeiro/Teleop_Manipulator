#include "teleop_mock/mock_robot.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace teleop {

bool MockRobot::init(const RobotContext& ctx) {
  period_ = ctx.node.declare_parameter<double>("robot.period", 0.002);
  const auto mode =
      ctx.node.declare_parameter<std::string>("robot.mode", "position");
  mode_ = mode == "velocity" ? CommandMode::Velocity : CommandMode::Position;
  dof_ = ctx.dof;
  q_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));
  dq_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));
  return true;
}

bool MockRobot::connect() {
  connected_ = true;
  return true;
}

void MockRobot::disconnect() noexcept { connected_ = false; }

std::optional<JointState> MockRobot::read() {
  if (!connected_) {
    return std::nullopt;
  }
  return JointState{q_, dq_, Clock::now()};
}

bool MockRobot::write(const JointCommand& cmd) {
  if (!connected_ || cmd.value.size() != q_.size()) {
    return false;
  }
  if (cmd.mode == CommandMode::Position) {
    dq_ = (cmd.value - q_) / period_;
    q_ = cmd.value;
  } else {
    dq_ = cmd.value;
    q_ += dq_ * period_;
  }
  return true;
}

void MockRobot::stop() noexcept { dq_.setZero(); }

}  // namespace teleop

PLUGINLIB_EXPORT_CLASS(teleop::MockRobot, teleop::ManipulatorInterface)
