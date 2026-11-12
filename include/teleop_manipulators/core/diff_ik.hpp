// Differential IK (damped least squares) using the robot URDF.
//
// solve() returns joint velocities that move the TCP toward a target pose.
// Pinocchio is hidden behind a pimpl, so this header (and everything that
// includes it) does not depend on Pinocchio.
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "teleop_manipulators/core/types.hpp"

namespace teleop {

class DiffIK {
public:
  struct Params {
    std::string urdf_xml;  // URDF content (from robot_description)
    std::string tcp_frame; // frame name in the URDF, e.g. "tool0"
    // Optional manipulator base frame (e.g. the link the arm is mounted on).
    // Everything below it is kept; mobile-base joints above it are locked.
    // Empty: keep the whole chain to the TCP and only lock side branches.
    std::string base_frame;
    double damping{
        0.05}; // DLS lambda: higher = safer near singularities, less accurate
    double gain_linear{2.0};  // [1/s] P-gain on position error
    double gain_angular{2.0}; // [1/s] P-gain on orientation error
  };

  explicit DiffIK(const Params &params);
  ~DiffIK();
  DiffIK(DiffIK &&) noexcept; // allows to move the constructor. Move object to
                              // new one
  DiffIK &operator=(DiffIK &&) noexcept;

  // nodiscard -> always return a value when calling the function. It also
  // verifies if the value was discarded
  [[nodiscard]] Eigen::Isometry3d forwardKinematics(const Eigen::VectorXd &q);
  [[nodiscard]] Eigen::VectorXd solve(const Eigen::VectorXd &q,
                                      const Eigen::Isometry3d &target);

  [[nodiscard]] std::size_t dof() const noexcept;
  [[nodiscard]] Eigen::VectorXd lowerLimits() const;
  [[nodiscard]] Eigen::VectorXd upperLimits() const;
  [[nodiscard]] Eigen::VectorXd velocityLimits() const;
  [[nodiscard]] std::vector<std::string> jointNames() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace teleop
