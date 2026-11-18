#include "teleop_manipulators/core/pose_mapper.hpp"

#include <utility>

namespace teleop {

PoseMapper::PoseMapper(Params params) : params_(std::move(params)) {}

void PoseMapper::engage(const Eigen::Isometry3d &T_tool,
                        const Eigen::Isometry3d &T_tcp) {
  T_tool0_ = T_tool;
  T_tcp0_ = T_tcp;
  engaged_ = true;
}

void PoseMapper::disengage() noexcept { engaged_ = false; }

std::optional<Eigen::Isometry3d>
PoseMapper::target(const Eigen::Isometry3d &T_tool) const {
  if (!engaged_) {
    return std::nullopt;
  }
  const Eigen::Matrix3d &R_bo = params_.R_base_optitrack;

  // Tool motion since engage, expressed in the OptiTrack world frame.
  const Eigen::Vector3d dp = T_tool.translation() - T_tool0_.translation();
  const Eigen::Matrix3d dR = T_tool.linear() * T_tool0_.linear().transpose();

  // Same motion, expressed in the robot base frame, applied to the TCP.
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() =
      T_tcp0_.translation() + params_.translation_scale * (R_bo * dp);
  T.linear() = (R_bo * dR * R_bo.transpose()) * T_tcp0_.linear();
  return T;
}

} // namespace teleop
