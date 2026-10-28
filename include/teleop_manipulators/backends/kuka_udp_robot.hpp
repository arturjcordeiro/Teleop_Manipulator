// KUKA backend: wraps your existing UDP client.
#pragma once

#include "teleop_manipulators/backends/robot_factory.hpp"

namespace teleop {

class KukaUdpRobot final : public RobotInterface {
public:
  explicit KukaUdpRobot(const RobotConfig& config);
  ~KukaUdpRobot() override;

  bool connect() override;
  void disconnect() noexcept override;
  [[nodiscard]] std::optional<JointState> read() override;
  bool write(const JointCommand& cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override { return config_.mode; }
  [[nodiscard]] double period() const noexcept override { return config_.period; }
  [[nodiscard]] std::size_t dof() const noexcept override { return config_.dof; }

private:
  RobotConfig config_;
  // TODO: your UDP client, e.g. std::unique_ptr<KukaUdpClient> client_;
};

}  // namespace teleop
