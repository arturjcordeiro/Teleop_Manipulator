#include "teleop_manipulators/core/safety.hpp"

#include <stdexcept>
#include <utility>

namespace teleop {

Safety::Safety(Params params) : params_(std::move(params)) {
  const auto n = params_.max_joint_velocity.size();
  if (params_.lower_limits.size() != n || params_.upper_limits.size() != n) {
    throw std::invalid_argument("Safety: limit vectors must all have the same size");
  }
}

bool Safety::poseIsFresh(Clock::time_point stamp, Clock::time_point now) const {
  return (now - stamp) <= params_.pose_timeout;
}

void Safety::clampVelocity(Eigen::VectorXd& dq) const {
  // ratio > 1 means that joint is too fast; scale ALL joints by the worst ratio
  // so the TCP keeps moving in the same direction, just slower.
  const double worst = (dq.cwiseAbs().array() / params_.max_joint_velocity.array()).maxCoeff();
  if (worst > 1.0) {
    dq /= worst;
  }
}

void Safety::clampPosition(Eigen::VectorXd& q) const {
  const Eigen::VectorXd lo = params_.lower_limits.array() + params_.limit_margin;
  const Eigen::VectorXd hi = params_.upper_limits.array() - params_.limit_margin;
  q = q.cwiseMax(lo).cwiseMin(hi);
}

}  // namespace teleop
