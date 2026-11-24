#include "teleop_kuka_udp/kuka_udp_robot.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <thread>

namespace teleop {

namespace {

constexpr auto kStateTimeout = std::chrono::milliseconds(250);
constexpr auto kKeepalivePeriod = std::chrono::milliseconds(50);
constexpr auto kPingPeriod = std::chrono::milliseconds(20);
constexpr auto kDiscoveryTimeout = std::chrono::seconds(1);
constexpr int kMaxDrainPerCall = 256;

// SmartServoPTP streams telemetry to <client ip>:STATE_PORT (30301) and answers
// pings on the sender's port. Binding our socket to 30301 makes both arrive
// here.
constexpr std::uint16_t kLocalStatePort = 30301;

// KUKA A,B,C: R = Rz(A) * Ry(B) * Rx(C), with B in [-pi/2, pi/2].
// Eigen's eulerAngles(2,1,0) returns A in [0, pi] instead and can jump to the
// equivalent set (A+pi, pi-B, C+pi), which the robot sees as a huge B change.
Eigen::Vector3d kukaAbc(const Eigen::Matrix3d &R) {
  const double b = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  const double a = std::atan2(R(1, 0), R(0, 0));
  const double c = std::atan2(R(2, 1), R(2, 2));
  return {a, b, c};
}

} // namespace

bool KukaUdpRobot::init(const RobotContext &ctx) {
  ip_ = ctx.node.declare_parameter<std::string>("robot.ip", "");
  port_ = ctx.node.declare_parameter<int>("robot.port", 0);
  period_ = ctx.node.declare_parameter<double>("robot.period", 0.002);
  const auto mode =
      ctx.node.declare_parameter<std::string>("robot.mode", "position");
  mode_ = mode == "velocity" ? CommandMode::Velocity : CommandMode::Position;
  dof_ = ctx.dof;
  max_joint_step_ =
      ctx.node.get_parameter_or<double>("robot.max_joint_step", 0.1);
  kuka_command_ =
      ctx.node.declare_parameter<std::string>("robot.kuka_command", "joint");
  kuka_speed_rel_ =
      ctx.node.declare_parameter<double>("robot.kuka_speed_rel", 1.0);
  kuka_accel_rel_ =
      ctx.node.declare_parameter<double>("robot.kuka_accel_rel", 1.0);
  kuka_speed_mm_s_ =
      ctx.node.declare_parameter<double>("robot.kuka_speed_mm_s", 200.0);
  kuka_accel_mm_s2_ =
      ctx.node.declare_parameter<double>("robot.kuka_accel_mm_s2", 0.0);
  forward_kinematics_ = ctx.forward_kinematics;
  return true;
}

KukaUdpRobot::~KukaUdpRobot() { disconnect(); }

// --------------------------------------------------------------- connection
bool KukaUdpRobot::connect() {
  disconnect();
  last_error_.clear();

  if (ip_.empty()) {
    last_error_ = "robot.ip is empty";
    return false;
  }
  if (port_ <= 0 || port_ > 65535) {
    last_error_ = "robot.port must be 1..65535";
    return false;
  }
  if (dof_ != kuka::kDof) {
    last_error_ = "KUKA iiwa 14 expects 7 joints but the model has " +
                  std::to_string(dof_);
    return false;
  }

  sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    last_error_ = std::string("socket failed: ") + std::strerror(errno);
    return false;
  }

  // Non-blocking from the start: recv() in read() must never stall the loop.
  const int flags = ::fcntl(sockfd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    last_error_ = std::string("fcntl failed: ") + std::strerror(errno);
    disconnect();
    return false;
  }

  int reuse = 1;
  ::setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  // Bind to the robot's telemetry destination port (see kLocalStatePort).
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(kLocalStatePort);
  if (::bind(sockfd_, reinterpret_cast<const sockaddr *>(&local),
             sizeof(local)) < 0) {
    last_error_ = "bind to UDP " + std::to_string(kLocalStatePort) +
                  " failed: " + std::strerror(errno) +
                  " (another client running?)";
    disconnect();
    return false;
  }

  sockaddr_in peer{};
  peer.sin_family = AF_INET;
  peer.sin_port = htons(static_cast<std::uint16_t>(port_));
  if (::inet_pton(AF_INET, ip_.c_str(), &peer.sin_addr) != 1) {
    last_error_ = "invalid robot.ip: " + ip_;
    disconnect();
    return false;
  }
  peer_ip_ = peer.sin_addr.s_addr;
  peer_port_ = peer.sin_port;

  // Discovery: the Java app latches our address from any packet and then
  // streams state. Wait for the first reply so we never start the loop blind.
  sendPing();
  const auto deadline = Clock::now() + kDiscoveryTimeout;
  while (!have_state_ && Clock::now() < deadline) {
    drainReplies();
    if (!have_state_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  if (!have_state_) {
    last_error_ = "no reply from " + ip_ + ":" + std::to_string(port_) +
                  " (is SmartServoPTP running?)";
    disconnect();
    return false;
  }

  last_error_.clear();
  return true;
}

void KukaUdpRobot::disconnect() noexcept {
  if (sockfd_ >= 0) {
    ::close(sockfd_);
    sockfd_ = -1;
  }
  have_state_ = false;
  have_cmd_ = false;
  q_.fill(0.0);
  dq_.fill(0.0);
}

// --------------------------------------------------------------- state
void KukaUdpRobot::drainReplies() {
  if (sockfd_ < 0) {
    return;
  }
  std::uint8_t buf[kuka::kReplySize];
  kuka::Reply reply;
  kuka::Reply newest;
  bool got = false;

  // Read everything queued; only the newest reply updates the cached state.
  for (int i = 0; i < kMaxDrainPerCall; ++i) {
    const ssize_t n = ::recv(sockfd_, buf, sizeof(buf), 0);
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        last_error_ = std::string("recv failed: ") + std::strerror(errno);
      }
      break;
    }
    if (kuka::decodeReply(buf, static_cast<std::size_t>(n), reply)) {
      newest = reply;
      got = true;
    }
  }
  if (!got) {
    return;
  }

  // Velocity from the newest sample vs. the previously cached one (not between
  // packets of the same burst, which arrive microseconds apart).
  const auto now = Clock::now();
  if (have_state_) {
    const double dt = std::chrono::duration<double>(now - last_state_).count();
    if (dt > 1e-3) {
      for (std::size_t j = 0; j < kuka::kDof; ++j) {
        dq_[j] = (newest.q[j] - q_[j]) / dt;
      }
    }
  } else {
    dq_.fill(0.0);
  }

  q_ = newest.q;
  last_state_ = now;
  have_state_ = true;
}

std::optional<JointState> KukaUdpRobot::read() {
  if (sockfd_ < 0) {
    return std::nullopt;
  }

  drainReplies();
  if (!have_state_) {
    return std::nullopt;
  }

  const auto now = Clock::now();
  const auto age = now - last_state_;
  if (age > kKeepalivePeriod && now - last_ping_ > kPingPeriod) {
    // Fallback only: telemetry normally streams at 120 Hz on its own.
    sendPing();
  }
  if (age > kStateTimeout) {
    last_error_ =
        "no KUKA state for > " + std::to_string(kStateTimeout.count()) + " ms";
    return std::nullopt;
  }

  JointState state;
  state.q = Eigen::Map<const Eigen::VectorXd>(
      q_.data(), static_cast<Eigen::Index>(kuka::kDof));
  state.dq = Eigen::Map<const Eigen::VectorXd>(
      dq_.data(), static_cast<Eigen::Index>(kuka::kDof));
  state.stamp = last_state_;
  return state;
}

// --------------------------------------------------------------- command
bool KukaUdpRobot::write(const JointCommand &cmd) {
  if (sockfd_ < 0) {
    return false;
  }
  if (cmd.value.size() != static_cast<Eigen::Index>(kuka::kDof)) {
    last_error_ = "KUKA expects 7 joint values";
    return false;
  }
  if (!have_state_) {
    last_error_ = "cannot command without measured state";
    return false;
  }
  if (!have_cmd_) {
    q_cmd_ = q_; // seed the command from the measured position
    have_cmd_ = true;
  }

  std::array<double, kuka::kDof> target{};
  if (cmd.mode == CommandMode::Position) {
    for (std::size_t j = 0; j < kuka::kDof; ++j) {
      target[j] = cmd.value[static_cast<Eigen::Index>(j)];
    }
  } else {
    // Integrate from the last COMMANDED target, not the measured position:
    // integrating from q_ makes every target trail the robot and the arm
    // creeps. The lead over the measured position is bounded so a robot that
    // can't keep up doesn't make the integrator run away.
    const double max_lead = 0.5 * max_joint_step_;
    for (std::size_t j = 0; j < kuka::kDof; ++j) {
      const double t =
          q_cmd_[j] + cmd.value[static_cast<Eigen::Index>(j)] * period_;
      target[j] = std::clamp(t, q_[j] - max_lead, q_[j] + max_lead);
    }
  }

  // Last line of defence, same rule as the UR backend.
  double jump = 0.0;
  for (std::size_t j = 0; j < kuka::kDof; ++j) {
    jump = std::max(jump, std::abs(target[j] - q_[j]));
  }
  if (jump > max_joint_step_) {
    last_error_ =
        "rejected target: " + std::to_string(jump) + " rad from actual";
    stop();
    return false;
  }

  if (!sendTarget(target, kuka_command_ == "cartesian")) {
    return false;
  }
  q_cmd_ = target;
  return true;
}

bool KukaUdpRobot::sendTarget(const std::array<double, kuka::kDof> &q,
                              bool cartesian) {
  if (cartesian) {
    if (!forward_kinematics_) {
      last_error_ =
          "cartesian mode needs forward_kinematics (robot_description)";
      return false;
    }
    // FK must give the FLANGE pose in the robot BASE frame, in meters
    // (SmartServoPTP moves the flange frame).
    const Eigen::VectorXd qv = Eigen::Map<const Eigen::VectorXd>(
        q.data(), static_cast<Eigen::Index>(kuka::kDof));
    const Eigen::Isometry3d pose = forward_kinematics_(qv);
    const Eigen::Vector3d abc = kukaAbc(pose.linear());

    std::uint8_t buf[kuka::kCartesianPacketSize];
    kuka::encodeCartesian(pose.translation().x() * 1000.0,
                          pose.translation().y() * 1000.0,
                          pose.translation().z() * 1000.0, abc[0], abc[1],
                          abc[2], kuka_speed_mm_s_, kuka_accel_mm_s2_, buf);
    return sendPacket(buf, sizeof(buf));
  }

  std::uint8_t buf[kuka::kJointPacketSize];
  kuka::encodeJoint(q, kuka_speed_rel_, kuka_accel_rel_, buf);
  return sendPacket(buf, sizeof(buf));
}

void KukaUdpRobot::stop() noexcept {
  try {
    have_cmd_ = false; // next command re-seeds from the measured position
    if (sockfd_ < 0 || !have_state_) {
      return;
    }
    // Hold the measured position in the SAME mode that is streaming. A joint
    // packet during a cartesian stream would make the Java app switch
    // LIN -> PTP (servo stop + restart), and back again on the next command.
    const bool cartesian = kuka_command_ == "cartesian";
    if (!sendTarget(q_, cartesian) && cartesian) {
      sendTarget(q_, false); // no FK available: fall back to a joint hold
    }
  } catch (...) {
  }
}

// --------------------------------------------------------------- UDP helpers
bool KukaUdpRobot::sendPacket(const std::uint8_t *data, std::size_t len) {
  if (sockfd_ < 0) {
    return false;
  }
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = peer_port_;
  dst.sin_addr.s_addr = peer_ip_;

  const ssize_t n =
      ::sendto(sockfd_, data, len, 0, reinterpret_cast<const sockaddr *>(&dst),
               sizeof(dst));
  if (n != static_cast<ssize_t>(len)) {
    last_error_ = std::string("sendto failed: ") + std::strerror(errno);
    return false;
  }
  return true;
}

bool KukaUdpRobot::sendPing() {
  const std::uint8_t one = 0;
  last_ping_ = Clock::now();
  return sendPacket(&one, sizeof(one));
}

} // namespace teleop

PLUGINLIB_EXPORT_CLASS(teleop::KukaUdpRobot, teleop::ManipulatorInterface)
