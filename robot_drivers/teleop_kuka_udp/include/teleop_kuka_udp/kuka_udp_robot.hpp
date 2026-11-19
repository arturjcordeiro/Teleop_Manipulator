// KUKA iiwa 14 R820 backend: SmartServoPTP over UDP, non-blocking.
//
// Teleop latency rules:
//   - read() never blocks: it drains whatever is on the socket and returns the
//     newest cached state.
//   - no secondary state socket (the command socket already streams replies).
//   - one packet in, one packet out per control period.
#pragma once

#include "teleop_kuka_udp/kuka_protocol.hpp"
#include "teleop_manipulators_core/manipulator_interface.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace teleop {

class KukaUdpRobot final : public ManipulatorInterface {
public:
  KukaUdpRobot() = default;
  ~KukaUdpRobot() override;

  bool init(const RobotContext &ctx) override;
  bool connect() override;
  void disconnect() noexcept override;
  [[nodiscard]] std::optional<JointState> read() override;
  bool write(const JointCommand &cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override {
    return mode_;
  }
  [[nodiscard]] double period() const noexcept override { return period_; }
  [[nodiscard]] std::size_t dof() const noexcept override { return dof_; }
  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  bool sendPing();
  bool sendPacket(const std::uint8_t *data, std::size_t len);
  void drainReplies();
  bool sendTarget(const std::array<double, kuka::kDof> &q, bool cartesian);

  std::string ip_;
  int port_{0};
  double period_{0.002};
  CommandMode mode_{CommandMode::Position};
  std::size_t dof_{0};
  double max_joint_step_{0.1};
  std::string kuka_command_{"joint"};
  double kuka_speed_rel_{1.0};
  double kuka_accel_rel_{1.0};
  double kuka_speed_mm_s_{200.0};
  double kuka_accel_mm_s2_{0.0};
  std::function<Eigen::Isometry3d(const Eigen::VectorXd &)> forward_kinematics_;

  int sockfd_{-1};
  std::uint32_t peer_ip_{0};   // network byte order
  std::uint16_t peer_port_{0}; // network byte order
  std::array<double, kuka::kDof> q_{};
  std::array<double, kuka::kDof> dq_{};

  std::array<double, kuka::kDof> q_cmd_{}; // last commanded joint target
  bool have_cmd_ = false;
  Clock::time_point last_state_{};
  Clock::time_point last_ping_{};
  bool have_state_{false};
  std::string last_error_;
};

} // namespace teleop
