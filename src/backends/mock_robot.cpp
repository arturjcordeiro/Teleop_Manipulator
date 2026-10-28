#include "teleop_manipulators/backends/mock_robot.hpp"

namespace teleop {

MockRobot::MockRobot(const RobotConfig& config)
    : config_(config),
      q_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config.dof))),
      dq_(Eigen::VectorXd::Zero(static_cast<Eigen::Index>(config.dof))) {
  // TODO: set a non-singular start pose (all zeros is singular for a UR).
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
    dq_ = (cmd.value - q_) / config_.period;
    q_ = cmd.value;
  } else {
    dq_ = cmd.value;
    q_ += dq_ * config_.period;
  }
  return true;
}

void MockRobot::stop() noexcept { dq_.setZero(); }

}  // namespace teleop
