#include "teleop_manipulators/backends/ur_rtde_robot.hpp"

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

UrRtdeRobot::UrRtdeRobot(const RobotConfig &config) : config_(config) {}

UrRtdeRobot::~UrRtdeRobot() { disconnect(); }

// --------------------------------------------------------------- connection
bool UrRtdeRobot::connect() {
  // Run RTDE at the same rate as our control loop (500 Hz e-Series, 125 Hz CB3)
  // and give ur_rtde's internal threads real-time priority.
  const double frequency = 1.0 / config_.period;
  try {
    recv_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(
        config_.ip, frequency, std::vector<std::string>{}, false, false,
        config_.rt_priority);
    ctrl_ = std::make_unique<ur_rtde::RTDEControlInterface>(
        config_.ip, frequency, ur_rtde::RTDEControlInterface::FLAGS_DEFAULT,
        kUrCapPort, config_.rt_priority);
  } catch (const std::exception &e) {
    last_error_ = "UR connect failed (" + config_.ip + "): " + e.what();
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
    if (jump > config_.max_joint_step) {
      target = q + lead * (config_.max_joint_step / jump);
    }
    // servoJ ignores speed/acceleration; time = one control period.
    return ctrl_->servoJ(toStd(target), 0.0, 0.0, config_.period,
                         config_.servo_lookahead, config_.servo_gain);
  }
  return ctrl_->speedJ(toStd(cmd.value), config_.acceleration,
                       3.0 * config_.period);
}

void UrRtdeRobot::stop() noexcept {
  try {
    if (!ctrl_ || !ctrl_->isConnected()) {
      return;
    }
    if (config_.mode == CommandMode::Position) {
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
    ctrl_->waitPeriod(t0); // sleep + spin; dt = 1 / frequency_ = config_.period
  }
}

} // namespace teleop
