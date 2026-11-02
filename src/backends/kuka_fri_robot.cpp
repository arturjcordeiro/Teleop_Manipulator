#include "teleop_manipulators/backends/kuka_fri_robot.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

// KUKA FRI Client SDK (from the Sunrise project, or the lbr-stack "fri"
// package)
#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

namespace teleop {

namespace {

using Clock = std::chrono::steady_clock;
constexpr std::size_t kN = 7;
using Joints = std::array<double, kN>;

constexpr int kDefaultFriPort = 30200;
constexpr unsigned int kReceiveTimeoutMs = 100; // lets the thread exit cleanly
constexpr auto kConnectTimeout = std::chrono::seconds(30);
constexpr auto kStateTimeout = std::chrono::milliseconds(100);
// Targets older than this are ignored and the robot holds (teleop stalled).
constexpr auto kTargetTimeout = std::chrono::milliseconds(100);

// Per-joint velocity limit enforced every FRI cycle (rad/s):
// ~50% of the iiwa 14 R820 maximum (85, 85, 100, 75, 130, 135, 135 deg/s).
constexpr Joints kMaxJointVel = {0.74, 0.74, 0.87, 0.65, 1.13, 1.18, 1.18};

double seconds(Clock::duration d) {
  return std::chrono::duration<double>(d).count();
}

} // namespace

// ===========================================================================
// FRI client: runs in the FRI thread, called once per FRI cycle by the SDK.
// ===========================================================================
class FriTeleopClient : public KUKA::FRI::LBRClient {
public:
  struct Snapshot {
    Joints q{};
    Joints dq{};
    Clock::time_point stamp{};
    bool valid = false;
    KUKA::FRI::ESessionState session = KUKA::FRI::IDLE;
  };

  explicit FriTeleopClient(double teleop_period_s)
      : teleop_period_(std::max(teleop_period_s, 1e-3)) {}

  void onStateChange(KUKA::FRI::ESessionState /*old_state*/,
                     KUKA::FRI::ESessionState new_state) override {
    std::lock_guard<std::mutex> lk(mtx_);
    session_ = new_state;
    if (new_state != KUKA::FRI::COMMANDING_ACTIVE) {
      // Leaving commanding: forget everything, re-seed from IPO on return.
      have_cmd_ = false;
      have_target_ = false;
    }
  }

  void monitor() override {
    LBRClient::monitor();
    updateMeasured();
  }

  void waitForCommand() override {
    LBRClient::waitForCommand(); // commands the IPO position, as FRI requires
    updateMeasured();
  }

  void command() override {
    updateMeasured();
    const double dt = robotState().getSampleTime();
    const auto now = Clock::now();

    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_cmd_) {
      // The first command must match the IPO position exactly.
      std::copy_n(robotState().getIpoJointPosition(), kN, q_cmd_.begin());
      seg_start_ = q_cmd_;
      seg_end_ = q_cmd_;
      have_cmd_ = true;
    }

    // Linear interpolation across the teleop period, from where we were when
    // the target arrived to the target itself. Without a fresh target: hold.
    Joints desired = q_cmd_;
    if (have_target_ && now - target_stamp_ < kTargetTimeout) {
      const double alpha =
          std::min(1.0, seconds(now - seg_t0_) / teleop_period_);
      for (std::size_t j = 0; j < kN; ++j) {
        desired[j] = seg_start_[j] + alpha * (seg_end_[j] - seg_start_[j]);
      }
    }

    // Hard per-cycle velocity limit: FRI applies positions directly.
    for (std::size_t j = 0; j < kN; ++j) {
      const double max_step = kMaxJointVel[j] * dt;
      q_cmd_[j] += std::clamp(desired[j] - q_cmd_[j], -max_step, max_step);
    }
    robotCommand().setJointPosition(q_cmd_.data());
  }

  // ---- called from the teleop thread ----

  Snapshot snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    Snapshot s;
    s.q = q_meas_;
    s.dq = dq_meas_;
    s.stamp = meas_stamp_;
    s.valid = have_meas_;
    s.session = session_;
    return s;
  }

  void setTarget(const Joints &q) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_cmd_) {
      return; // not commanding yet; command() will seed from IPO
    }
    const auto now = Clock::now();
    seg_start_ = q_cmd_; // continue from the current commanded position
    seg_end_ = q;
    seg_t0_ = now;
    target_stamp_ = now;
    have_target_ = true;
  }

  void clearTarget() {
    std::lock_guard<std::mutex> lk(mtx_);
    have_target_ = false; // command() holds the current commanded position
  }

private:
  void updateMeasured() {
    const double *q = robotState().getMeasuredJointPosition();
    const double dt = robotState().getSampleTime();
    const auto now = Clock::now();

    std::lock_guard<std::mutex> lk(mtx_);
    for (std::size_t j = 0; j < kN; ++j) {
      dq_meas_[j] = (have_meas_ && dt > 0.0) ? (q[j] - q_meas_[j]) / dt : 0.0;
      q_meas_[j] = q[j];
    }
    meas_stamp_ = now;
    have_meas_ = true;
  }

  const double teleop_period_;

  mutable std::mutex mtx_;
  KUKA::FRI::ESessionState session_ = KUKA::FRI::IDLE;

  Joints q_meas_{};
  Joints dq_meas_{};
  Clock::time_point meas_stamp_{};
  bool have_meas_ = false;

  Joints q_cmd_{};
  bool have_cmd_ = false;

  Joints seg_start_{};
  Joints seg_end_{};
  Clock::time_point seg_t0_{};
  Clock::time_point target_stamp_{};
  bool have_target_ = false;
};

// ===========================================================================
// Backend
// ===========================================================================
KukaFriRobot::KukaFriRobot(const RobotConfig &config) : config_(config) {}

KukaFriRobot::~KukaFriRobot() { disconnect(); }

bool KukaFriRobot::connect() {
  disconnect();
  last_error_.clear();

  if (config_.dof != static_cast<int>(kDof)) {
    last_error_ = "KUKA iiwa 14 expects 7 joints but the model has " +
                  std::to_string(config_.dof);
    return false;
  }
  if (config_.kuka_fri_ip.empty()) {
    last_error_ = "robot.fri_ip is empty (use the KONI IP, e.g. 192.170.10.2)";
    return false;
  }

  connection_ = std::make_unique<KUKA::FRI::UdpConnection>(kReceiveTimeoutMs);
  client_ = std::make_unique<FriTeleopClient>(config_.period);
  app_ = std::make_unique<KUKA::FRI::ClientApplication>(*connection_, *client_);

  const int port =
      config_.kuka_fri_port > 0 ? config_.kuka_fri_port : kDefaultFriPort;
  if (!app_->connect(port, config_.kuka_fri_ip.c_str())) {
    last_error_ = "FRI connect to " + config_.kuka_fri_ip + ":" +
                  std::to_string(port) + " failed";
    disconnect();
    return false;
  }

  // FRI thread: step() blocks until the next robot packet (or the receive
  // timeout) and calls monitor()/waitForCommand()/command().
  // For 5 ms reliably, give this thread realtime priority (SCHED_FIFO) and
  // ideally run a PREEMPT_RT kernel.
  running_ = true;
  thread_ = std::thread([this] {
    while (running_) {
      app_->step();
    }
  });

  // Wait until Iiwa_ros2 has started the session in a commanding mode.
  const auto deadline = Clock::now() + kConnectTimeout;
  while (Clock::now() < deadline) {
    const auto s = client_->snapshot();
    if (s.valid && s.session == KUKA::FRI::COMMANDING_ACTIVE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  last_error_ =
      "FRI session not COMMANDING_ACTIVE within " +
      std::to_string(kConnectTimeout.count()) +
      " s (start Iiwa_ros2 in a POSITION mode, check the KONI network)";
  disconnect();
  return false;
}

void KukaFriRobot::disconnect() noexcept {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  if (app_) {
    app_->disconnect();
  }
  app_.reset();
  client_.reset();
  connection_.reset();
  have_cmd_ = false;
}

std::optional<JointState> KukaFriRobot::read() {
  if (!client_) {
    return std::nullopt;
  }
  const auto s = client_->snapshot();
  if (!s.valid) {
    return std::nullopt;
  }
  if (Clock::now() - s.stamp > kStateTimeout) {
    last_error_ =
        "no FRI state for > " + std::to_string(kStateTimeout.count()) + " ms";
    return std::nullopt;
  }

  JointState state;
  state.q = Eigen::Map<const Eigen::VectorXd>(s.q.data(),
                                              static_cast<Eigen::Index>(kDof));
  state.dq = Eigen::Map<const Eigen::VectorXd>(s.dq.data(),
                                               static_cast<Eigen::Index>(kDof));
  state.stamp = s.stamp; // adjust if JointState uses a different clock
  return state;
}

bool KukaFriRobot::write(const JointCommand &cmd) {
  if (!client_) {
    return false;
  }
  if (cmd.value.size() != static_cast<Eigen::Index>(kDof)) {
    last_error_ = "KUKA expects 7 joint values";
    return false;
  }
  const auto s = client_->snapshot();
  if (!s.valid || s.session != KUKA::FRI::COMMANDING_ACTIVE) {
    last_error_ = "FRI session is not commanding";
    have_cmd_ = false;
    return false;
  }
  if (!have_cmd_) {
    q_cmd_ = s.q;
    have_cmd_ = true;
  }

  Joints target{};
  if (cmd.mode == CommandMode::Position) {
    for (std::size_t j = 0; j < kDof; ++j) {
      target[j] = cmd.value[static_cast<Eigen::Index>(j)];
    }
  } else {
    // Integrate from the last commanded target, bounded lead over measured.
    const double max_lead = 0.5 * config_.max_joint_step;
    for (std::size_t j = 0; j < kDof; ++j) {
      const double t =
          q_cmd_[j] + cmd.value[static_cast<Eigen::Index>(j)] * config_.period;
      target[j] = std::clamp(t, s.q[j] - max_lead, s.q[j] + max_lead);
    }
  }

  double jump = 0.0;
  for (std::size_t j = 0; j < kDof; ++j) {
    jump = std::max(jump, std::abs(target[j] - s.q[j]));
  }
  if (jump > config_.max_joint_step) {
    last_error_ =
        "rejected target: " + std::to_string(jump) + " rad from actual";
    stop();
    return false;
  }

  client_->setTarget(target);
  q_cmd_ = target;
  return true;
}

void KukaFriRobot::stop() noexcept {
  have_cmd_ = false;
  if (client_) {
    client_->clearTarget(); // FRI thread keeps sending the held position
  }
}

} // namespace teleop
