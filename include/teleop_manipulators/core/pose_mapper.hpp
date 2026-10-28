// Clutch + relative mapping: OptiTrack tool pose -> desired TCP pose.
//
// On engage(), stores the tool pose and the current TCP pose.
// Afterwards, tool motion (expressed in the OptiTrack frame) is rotated into the
// robot base frame, scaled, and applied to the stored TCP pose.
#pragma once

#include <optional>

#include "teleop_manipulators/core/types.hpp"

namespace teleop {

class PoseMapper {
public:
  struct Params {
    Eigen::Matrix3d R_base_optitrack{Eigen::Matrix3d::Identity()};  // OptiTrack -> robot base
    double translation_scale{1.0};
  };

  explicit PoseMapper(Params params);

  void engage(const Eigen::Isometry3d& T_tool, const Eigen::Isometry3d& T_tcp);
  void disengage() noexcept;
  [[nodiscard]] bool engaged() const noexcept { return engaged_; }

  // Desired TCP pose in the robot base frame, or nullopt if not engaged.
  [[nodiscard]] std::optional<Eigen::Isometry3d> target(const Eigen::Isometry3d& T_tool) const;

private:
  Params params_;
  bool engaged_{false};
  Eigen::Isometry3d T_tool0_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d T_tcp0_{Eigen::Isometry3d::Identity()};
};

}  // namespace teleop
