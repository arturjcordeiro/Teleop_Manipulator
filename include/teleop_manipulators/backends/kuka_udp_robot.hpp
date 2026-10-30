// KUKA iiwa 14 R820 backend: SmartServoPTP over UDP, non-blocking.
//
// Teleop latency rules:
//   - read() never blocks: it drains whatever is on the socket and returns the
//     newest cached state.
//   - no secondary state socket (the command socket already streams replies).
//   - one packet in, one packet out per control period.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "teleop_manipulators/backends/kuka_protocol.hpp"
#include "teleop_manipulators/backends/robot_factory.hpp"

namespace teleop {

class KukaUdpRobot final : public RobotInterface {
public:
  explicit KukaUdpRobot(const RobotConfig &config);
  ~KukaUdpRobot() override;

  bool connect() override;
  void disconnect() noexcept override;
  [[nodiscard]] std::optional<JointState> read() override;
  bool write(const JointCommand &cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override {
    return config_.mode;
  }
  [[nodiscard]] double period() const noexcept override {
    return config_.period;
  }
  [[nodiscard]] std::size_t dof() const noexcept override { return config_.dof; }
  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  bool sendPing();
  bool sendPacket(const std::uint8_t *data, std::size_t len);
  void drainReplies();
  bool sendTarget(const std::array<double, kuka::kDof> &q, bool cartesian);

  RobotConfig config_;
  int sockfd_{-1};
  std::uint32_t peer_ip_{0}; // network byte order
  std::uint16_t peer_port_{0}; // network byte order
  std::array<double, kuka::kDof> q_{};
  std::array<double, kuka::kDof> dq_{};
  Clock::time_point last_state_{};
  Clock::time_point last_ping_{};
  bool have_state_{false};
  std::string last_error_;
};

} // namespace teleop
