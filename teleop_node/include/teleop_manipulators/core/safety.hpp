// Safety checks applied every control cycle, independent of the robot.
#pragma once

#include <chrono>

#include "teleop_manipulators_core/types.hpp"

namespace teleop {

class Safety {
public:
  struct Params {
    Eigen::VectorXd max_joint_velocity;  // [rad/s]
    Eigen::VectorXd lower_limits;        // [rad]
    Eigen::VectorXd upper_limits;        // [rad]
    double limit_margin{0.05};           // [rad] stay this far from hard limits
    std::chrono::milliseconds pose_timeout{50};
  };

  explicit Safety(Params params);

  // False if OptiTrack data is too old (tracking lost, network drop...).
  [[nodiscard]] bool poseIsFresh(Clock::time_point stamp, Clock::time_point now) const;

  // Scales dq uniformly so no joint exceeds its limit (keeps the direction).
  void clampVelocity(Eigen::VectorXd& dq) const;

  // Clamps a position command inside the joint limits (minus margin).
  void clampPosition(Eigen::VectorXd& q) const;

private:
  Params params_;
};

}  // namespace teleop
