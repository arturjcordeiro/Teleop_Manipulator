// Universal Robots backend via ur_rtde, tuned for teleoperation.
//   Position mode -> servoJ (recommended: smooth, filtered by lookahead)
//   Velocity mode -> speedJ
#pragma once

#include <memory>
#include <string>

#include "teleop_manipulators/backends/robot_factory.hpp"

// Forward declarations: keep ur_rtde headers out of this header.
namespace ur_rtde {
class RTDEControlInterface;
class RTDEReceiveInterface;
} // namespace ur_rtde

namespace teleop {

class UrRtdeRobot final : public RobotInterface {
public:
  explicit UrRtdeRobot(const RobotConfig &config);
  ~UrRtdeRobot() override; // defined in .cpp where ur_rtde types are complete

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
  [[nodiscard]] std::size_t dof() const noexcept override { return 6; }

  [[nodiscard]] std::string lastError() const override { return last_error_; }

private:
  // True if the robot can accept motion commands right now.
  [[nodiscard]] bool ready();

  RobotConfig config_;
  std::unique_ptr<ur_rtde::RTDEControlInterface> ctrl_;
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> recv_;
  std::string last_error_;
};

} // namespace teleop
