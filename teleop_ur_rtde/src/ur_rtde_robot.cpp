#include "teleop_ur_rtde/ur_rtde_robot.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <exception>
#include <vector>

namespace teleop {

namespace {
std::vector<double> toStd(const Eigen::VectorXd &v) {
  return {v.data(), v.data() + v.size()};
}

Eigen::VectorXd toEigen(const std::vector<double> &v) {
  return Eigen::Map<const Eigen::VectorXd>(v.data(),
                                           static_cast<Eigen::Index>(v.size()));
}

constexpr int kUrCapPort = 50002; // ur_rtde default
} // namespace

bool UrRtdeRobot::init(const RobotContext &ctx) {
  ip_ = ctx.node.declare_parameter<std::string>("robot.ip", "");
  period_ = ctx.node.declare_parameter<double>("robot.period", 0.002);
  const auto mode =
      ctx.node.declare_parameter<std::string>("robot.mode", "position");
  mode_ = mode == "velocity" ? CommandMode::Velocity : CommandMode::Position;
  max_joint_step_ =
      ctx.node.get_parameter_or<double>("robot.max_joint_step", 0.1);
  servo_lookahead_ =
      ctx.node.declare_parameter<double>("robot.servo_lookahead", 0.1);
  servo_gain_ = ctx.node.declare_parameter<double>("robot.servo_gain", 300.0);
  acceleration_ = ctx.node.declare_parameter<double>("robot.acceleration", 1.0);
  rt_priority_ = ctx.node.declare_parameter<int>("robot.rt_priority", 80);
  return true;
}

UrRtdeRobot::~UrRtdeRobot() { disconnect(); }

// --------------------------------------------------------------- connection
bool UrRtdeRobot::connect() {
  // Run RTDE at the same rate as our control loop (500 Hz e-Series, 125 Hz CB3)
  // and give ur_rtde's internal threads real-time priority.
  const double frequency = 1.0 / period_;
  try {
    recv_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(
        ip_, frequency, std::vector<std::string>{}, false, false,
        rt_priority_);
    ctrl_ = std::make_unique<ur_rtde::RTDEControlInterface>(
        ip_, frequency, ur_rtde::RTDEControlInterface::FLAGS_DEFAULT,
        kUrCapPort, rt_priority_);
  } catch (const std::exception &e) {
    last_error_ = "UR connect failed (" + ip_ + "): " + e.what();
    ctrl_.reset();
    recv_.reset();
    return false;
  }
  return ready();
}

void UrRtdeRobot::disconnect() noexcept {
  try {
    if (ctrl_) {
      stop();
      ctrl_->stopScript();
      ctrl_->disconnect();
    }
    if (recv_) {
      recv_->disconnect();
    }
  } catch (...) {
  }
  ctrl_.reset();
  recv_.reset();
}

bool UrRtdeRobot::ready() {
  if (!ctrl_ || !recv_) {
    last_error_ = "not connected";
    return false;
  }
  if (!recv_->isConnected() || !ctrl_->isConnected()) {
    last_error_ = "RTDE connection lost";
    return false;
  }
  if (recv_->isEmergencyStopped()) {
    last_error_ = "emergency stop active";
    return false;
  }
  if (recv_->isProtectiveStopped()) {
    last_error_ = "protective stop: unlock on the pendant, then reconnect";
    return false;
  }
  if (!ctrl_->isProgramRunning()) {
    last_error_ = "control script not running (stopped from the pendant?)";
    return false;
  }
  return true;
}

// --------------------------------------------------------------- state
std::optional<JointState> UrRtdeRobot::read() {
  if (!ready()) {
    return std::nullopt; // control loop disengages and stops
  }
  return JointState{toEigen(recv_->getActualQ()), toEigen(recv_->getActualQd()),
                    Clock::now()};
}

// --------------------------------------------------------------- command
bool UrRtdeRobot::write(const JointCommand &cmd) {
  if (!ready()) {
    return false;
  }
  if (cmd.value.size() != 6) {
    last_error_ = "UR expects 6 joint values";
    return false;
  }

  if (cmd.mode == CommandMode::Position) {
    // Bound the lead over the measured joints. Rejecting a far target with
    // servoStop() braked the robot during fast teleop, where the servoJ target
    // legitimately leads the lagging arm. Scaling the whole vector keeps the
    // motion direction and lets it continue; the lead stays hard-bounded.
    const Eigen::VectorXd q = toEigen(recv_->getActualQ());
    Eigen::VectorXd target = cmd.value;
    const Eigen::VectorXd lead = target - q;
    const double jump = lead.cwiseAbs().maxCoeff();
    if (jump > max_joint_step_) {
      target = q + lead * (max_joint_step_ / jump);
    }
    // servoJ ignores speed/acceleration; time = one control period.
    return ctrl_->servoJ(toStd(target), 0.0, 0.0, period_, servo_lookahead_,
                         servo_gain_);
  }
  return ctrl_->speedJ(toStd(cmd.value), acceleration_, 3.0 * period_);
}

void UrRtdeRobot::stop() noexcept {
  try {
    if (!ctrl_ || !ctrl_->isConnected()) {
      return;
    }
    if (mode_ == CommandMode::Position) {
      ctrl_->servoStop();
    } else {
      ctrl_->speedStop();
    }
  } catch (...) {
  }
}

UrRtdeRobot::CycleToken UrRtdeRobot::beginCycle() {
  return ctrl_ ? ctrl_->initPeriod() : Clock::now();
}

void UrRtdeRobot::endCycle(CycleToken t0) {
  if (ctrl_) {
    ctrl_->waitPeriod(t0); // sleep + spin; dt = 1 / frequency_ = period_
  }
}

} // namespace teleop

PLUGINLIB_EXPORT_CLASS(teleop::UrRtdeRobot, teleop::ManipulatorInterface)
