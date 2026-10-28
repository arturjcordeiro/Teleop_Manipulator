#include "teleop_manipulators/core/diff_ik.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <stdexcept>

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
  pinocchio::urdf::buildModelFromXML(params.urdf_xml, impl_->model);
  impl_->data = pinocchio::Data(impl_->model);

  if (!impl_->model.existFrame(params.tcp_frame)) {
    throw std::invalid_argument("TCP frame '" + params.tcp_frame + "' not found in URDF");
  }
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
