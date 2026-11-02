// Creates the right backend from a config. The node calls only this,
// so adding a robot = one new backend class + one line in the factory.
#pragma once

#include <functional>
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
      0.1}; // [rad] reject targets further than this from actual q

  // KUKA interface selector and FRI address (ignored by other backends)
  std::string kuka_interface{"udp"}; // "udp" (SmartServoPTP) | "fri"
  std::string kuka_fri_ip{"192.170.10.2"};
  int kuka_fri_port{30200};

  // KUKA SmartServoPTP (ignored by other backends)
  std::string kuka_command{"joint"}; // "joint" (72 B) | "cartesian" (64 B)
  double kuka_speed_rel{1.0};        // joint PTP speed scale [0..1]
  double kuka_accel_rel{1.0};        // joint PTP acceleration scale [0..1]
  double kuka_speed_mm_s{200.0};     // cartesian LIN speed
  double kuka_accel_mm_s2{0.0};      // cartesian LIN acceleration
  // FK for cartesian mode. The node injects DiffIK::forwardKinematics so the
  // driver does not build a second pinocchio model.
  std::function<Eigen::Isometry3d(const Eigen::VectorXd &)> forward_kinematics;
};

[[nodiscard]] std::unique_ptr<RobotInterface>
makeRobot(const RobotConfig &config);

} // namespace teleop
