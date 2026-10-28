// Creates the right backend from a config. The node calls only this,
// so adding a robot = one new backend class + one line in the factory.
#pragma once

#include <memory>
#include <string>

#include "teleop_manipulators/core/robot_interface.hpp"

namespace teleop {

struct RobotConfig {
  std::string type{"mock"}; // "mock" | "ur" | "kuka"
  std::string ip;
  int port{0};
  double period{0.002}; // [s] control period
  CommandMode mode{CommandMode::Position};
  std::size_t dof{6};

  // UR servoJ tuning (ignored by other backends)
  double servo_lookahead{0.1}; // [s] 0.03..0.2, higher = smoother but laggier
  double servo_gain{300.0};    // 100..2000
  double acceleration{1.0};    // [rad/s^2] for speedJ / stop
  //
  int rt_priority{80}; // ur_rtde thread priority (needs rtprio permission)
  double max_joint_step{
      0.1}; // [rad] reject servoJ targets further than this from actual q
};

[[nodiscard]] std::unique_ptr<RobotInterface>
makeRobot(const RobotConfig &config);

} // namespace teleop
