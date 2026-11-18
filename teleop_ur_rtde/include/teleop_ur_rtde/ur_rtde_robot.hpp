// Universal Robots backend via ur_rtde, tuned for teleoperation.
//   Position mode -> servoJ (recommended: smooth, filtered by lookahead)
//   Velocity mode -> speedJ
#pragma once

#include "teleop_manipulators_core/manipulator_interface.hpp"

#include <memory>
#include <string>

// Forward declarations: keep ur_rtde headers out of this header.
namespace ur_rtde {
class RTDEControlInterface;
class RTDEReceiveInterface;
} // namespace ur_rtde

namespace teleop {

class UrRtdeRobot final : public ManipulatorInterface {
public:
  UrRtdeRobot() = default;
  ~UrRtdeRobot() override; // defined in .cpp where ur_rtde types are complete

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
  [[nodiscard]] std::size_t dof() const noexcept override { return 6; }

  [[nodiscard]] CycleToken beginCycle() override;
  void endCycle(CycleToken t0) override;

  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  // True if the robot can accept motion commands right now.
  [[nodiscard]] bool ready();

  std::string ip_;
  double period_{0.002};
  CommandMode mode_{CommandMode::Position};
  double max_joint_step_{0.1};
  double servo_lookahead_{0.1};
  double servo_gain_{300.0};
  double acceleration_{1.0};
  int rt_priority_{80};

  std::unique_ptr<ur_rtde::RTDEControlInterface> ctrl_;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> recv_;
  std::string last_error_;
};

} // namespace teleop
