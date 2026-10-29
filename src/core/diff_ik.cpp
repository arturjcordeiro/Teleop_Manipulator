#include "teleop_manipulators/core/diff_ik.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <stdexcept>
#include <vector>

namespace teleop {

struct DiffIK::Impl {
  Params params;
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::FrameIndex tcp_id{};
  Eigen::MatrixXd J;  // preallocated 6 x nv
};

DiffIK::DiffIK(const Params& params) : impl_(std::make_unique<Impl>()) {
  impl_->params = params;

  pinocchio::Model full;
  pinocchio::urdf::buildModelFromXML(params.urdf_xml, full);

  if (!full.existFrame(params.tcp_frame)) {
    throw std::invalid_argument("TCP frame '" + params.tcp_frame + "' not found in URDF");
  }
  const auto tcp_frame_id = full.getFrameId(params.tcp_frame);

  // Keep only the joints on the path from the manipulator base (or the URDF
  // root) down to the TCP. Mobile-base joints (wheels, casters, ...) hang off
  // side branches and must not take part in the IK.
  pinocchio::JointIndex start = 0;
  if (!params.base_frame.empty()) {
    if (!full.existFrame(params.base_frame)) {
      throw std::invalid_argument("Base frame '" + params.base_frame +
                                  "' not found in URDF");
    }
    start = full.frames[full.getFrameId(params.base_frame)].parentJoint;
  }

  std::vector<bool> keep(static_cast<std::size_t>(full.njoints), false);
  bool reached_start = (start == 0);
  for (auto j = full.frames[tcp_frame_id].parentJoint; j != 0; j = full.parents[j]) {
    if (j == start) {
      reached_start = true;
      break;
    }
    keep[j] = true;
  }
  if (!reached_start) {
    throw std::invalid_argument("Base frame '" + params.base_frame +
                                "' is not on the kinematic path to TCP frame '" +
                                params.tcp_frame + "'");
  }

  std::vector<pinocchio::JointIndex> locks;
  for (std::size_t j = 1; j < static_cast<std::size_t>(full.njoints); ++j) {
    if (!keep[j] && full.joints[j].nv() > 0) {
      locks.push_back(j);
    }
  }

  if (locks.empty()) {
    impl_->model = full;
  } else {
    impl_->model =
        pinocchio::buildReducedModel(full, locks, pinocchio::neutral(full));
  }

  if (!impl_->model.existFrame(params.tcp_frame)) {
    throw std::invalid_argument("TCP frame '" + params.tcp_frame +
                                "' was pruned from the URDF model");
  }
  impl_->data = pinocchio::Data(impl_->model);
  impl_->tcp_id = impl_->model.getFrameId(params.tcp_frame);
  impl_->J = Eigen::MatrixXd::Zero(6, impl_->model.nv);
}

DiffIK::~DiffIK() = default;
DiffIK::DiffIK(DiffIK&&) noexcept = default;
DiffIK& DiffIK::operator=(DiffIK&&) noexcept = default;

Eigen::Isometry3d DiffIK::forwardKinematics(const Eigen::VectorXd& q) {
  auto& m = impl_->model;
  auto& d = impl_->data;
  pinocchio::forwardKinematics(m, d, q);
  pinocchio::updateFramePlacement(m, d, impl_->tcp_id);

  const auto& M = d.oMf[impl_->tcp_id];
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = M.rotation();
  T.translation() = M.translation();
  return T;
}

Eigen::VectorXd DiffIK::solve(const Eigen::VectorXd& q, const Eigen::Isometry3d& target) {
  auto& m = impl_->model;
  auto& d = impl_->data;
  const auto& p = impl_->params;

  // Kinematics + Jacobians at the current configuration.
  pinocchio::computeJointJacobians(m, d, q);
  pinocchio::updateFramePlacement(m, d, impl_->tcp_id);

  // Pose error expressed in the TCP (LOCAL) frame: [v; w].
  const pinocchio::SE3 current = d.oMf[impl_->tcp_id];
  const pinocchio::SE3 desired(target.linear(), target.translation());
  const Eigen::Matrix<double, 6, 1> err = pinocchio::log6(current.actInv(desired)).toVector();

  // Desired TCP twist (P-controller).
  Eigen::Matrix<double, 6, 1> twist;
  twist.head<3>() = p.gain_linear * err.head<3>();
  twist.tail<3>() = p.gain_angular * err.tail<3>();

  // Jacobian in the same (LOCAL) frame as the error.
  impl_->J.setZero();
  pinocchio::getFrameJacobian(m, d, impl_->tcp_id, pinocchio::LOCAL, impl_->J);
  const Eigen::MatrixXd& J = impl_->J;

  // Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 * twist
  const Eigen::Matrix<double, 6, 6> A =
      J * J.transpose() + p.damping * p.damping * Eigen::Matrix<double, 6, 6>::Identity();
  return J.transpose() * A.ldlt().solve(twist);
}

std::size_t DiffIK::dof() const noexcept { return static_cast<std::size_t>(impl_->model.nv); }
Eigen::VectorXd DiffIK::lowerLimits() const { return impl_->model.lowerPositionLimit; }
Eigen::VectorXd DiffIK::upperLimits() const { return impl_->model.upperPositionLimit; }
Eigen::VectorXd DiffIK::velocityLimits() const { return impl_->model.velocityLimit; }

}  // namespace teleop
