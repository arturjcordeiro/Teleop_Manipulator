#include "teleop_manipulators/backends/robot_factory.hpp"

#include <stdexcept>

#include "teleop_manipulators/backends/kuka_udp_robot.hpp"
#include "teleop_manipulators/backends/mock_robot.hpp"
#ifdef TELEOP_HAS_UR
#include "teleop_manipulators/backends/ur_rtde_robot.hpp"
#endif

namespace teleop {

std::unique_ptr<RobotInterface> makeRobot(const RobotConfig& config) {
  if (config.type == "mock") {
    return std::make_unique<MockRobot>(config);
  }
  if (config.type == "kuka") {
    return std::make_unique<KukaUdpRobot>(config);
  }
  if (config.type == "ur") {
#ifdef TELEOP_HAS_UR
    return std::make_unique<UrRtdeRobot>(config);
#else
    throw std::runtime_error("UR backend not built (ur_rtde was not found at build time)");
#endif
  }
  throw std::invalid_argument("Unknown robot type: " + config.type);
}

}  // namespace teleop
