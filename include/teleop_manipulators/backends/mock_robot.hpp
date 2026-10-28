// Fake robot: integrates the commands it receives. Lets you test the whole
// pipeline (OptiTrack -> mapping -> IK -> safety) without hardware.
#pragma once

#include "teleop_manipulators/backends/robot_factory.hpp"

namespace teleop {

class MockRobot final : public RobotInterface {
public:
  explicit MockRobot(const RobotConfig& config);

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
  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  bool connected_{false};
};

}  // namespace teleop
