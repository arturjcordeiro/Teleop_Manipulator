// Fake robot: integrates the commands it receives. Lets you test the whole
// pipeline (OptiTrack -> mapping -> IK -> safety) without hardware.
#pragma once

#include "teleop_manipulators_core/manipulator_interface.hpp"

namespace teleop {

class MockRobot final : public ManipulatorInterface {
public:
  MockRobot() = default;

  bool init(const RobotContext& ctx) override;
  bool connect() override;
  void disconnect() noexcept override;
  [[nodiscard]] std::optional<JointState> read() override;
  bool write(const JointCommand& cmd) override;
  void stop() noexcept override;

  [[nodiscard]] CommandMode commandMode() const noexcept override { return mode_; }
  [[nodiscard]] double period() const noexcept override { return period_; }
  [[nodiscard]] std::size_t dof() const noexcept override { return dof_; }

private:
  double period_{0.002};
  CommandMode mode_{CommandMode::Position};
  std::size_t dof_{0};
  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  bool connected_{false};
};

}  // namespace teleop
