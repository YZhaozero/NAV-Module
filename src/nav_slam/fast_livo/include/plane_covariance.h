#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <limits>

namespace fast_livo_plane {

struct Fit {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> uncertainty = Eigen::Matrix<double, 6, 6>::Zero();
  bool valid = false;
};

// Points expose point_w and var. Keeping this operation independent of the
// octree permits numerical comparison against the original plane estimator.
template <typename Points>
Fit estimate(const Points &points, double plane_threshold) {
  Fit result;
  if (points.size() < 3) return result;
  const double count = static_cast<double>(points.size());
  const double inverse_count = 1. / count;
  for (const auto &point : points) {
    result.center += point.point_w;
    result.covariance.noalias() += point.point_w * point.point_w.transpose();
  }
  // Preserve the existing covariance convention and summation order. A change
  // to incremental/centered moments requires its own numerical comparison.
  result.center /= count;
  result.covariance = result.covariance / count - result.center * result.center.transpose();
  if (!result.covariance.allFinite()) return result;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result.covariance);
  if (solver.info() != Eigen::Success) return result;
  result.eigenvalues = solver.eigenvalues();  // Ascending, including equal values.
  result.eigenvectors = solver.eigenvectors();
  if (!result.eigenvalues.allFinite() || !result.eigenvectors.allFinite()) return result;
  const double numerical_gap = 64. * std::numeric_limits<double>::epsilon() *
                               result.eigenvalues.cwiseAbs().maxCoeff();
  // Duplicate/collinear/isotropic points do not define a unique plane normal.
  // Do not divide by a zero eigenvalue gap or index an artificial middle column
  // when minCoeff and maxCoeff select the same index. Equal tangent eigenvalues
  // ARE valid; only the minimum must be unique at floating-point resolution.
  if (result.eigenvalues[2] <= 0. ||
      result.eigenvalues[1] - result.eigenvalues[0] <= numerical_gap ||
      result.eigenvalues[0] >= plane_threshold) return result;

  const Eigen::Vector3d normal = result.eigenvectors.col(0);
  Eigen::Matrix3d normal_operator = Eigen::Matrix3d::Zero();
  for (int axis = 1; axis < 3; ++axis) {
    normal_operator.noalias() +=
        result.eigenvectors.col(axis) * result.eigenvectors.col(axis).transpose() /
        (count * (result.eigenvalues[0] - result.eigenvalues[axis]));
  }
  // The eigenvectors and eigenvalue gaps are constant for the complete plane.
  // Algebraically factor the old evecs * F expression instead of constructing
  // two 3x3 eigenvector products for every observation.
  for (const auto &point : points) {
    const Eigen::Vector3d delta = point.point_w - result.center;
    const Eigen::Matrix3d jacobian = normal_operator * delta * normal.transpose() +
                                     delta.dot(normal) * normal_operator;
    const Eigen::Matrix3d weighted = jacobian * point.var;
    result.uncertainty.template topLeftCorner<3, 3>().noalias() += weighted * jacobian.transpose();
    result.uncertainty.template topRightCorner<3, 3>() += weighted * inverse_count;
    // Preserve even a slightly asymmetric input state covariance; do not assume
    // the other cross block is its exact floating-point transpose.
    result.uncertainty.template bottomLeftCorner<3, 3>().noalias() +=
        (point.var * jacobian.transpose()) * inverse_count;
    result.uncertainty.template bottomRightCorner<3, 3>() +=
        point.var * (inverse_count * inverse_count);
  }
  result.valid = result.uncertainty.allFinite();
  if (!result.valid) result.uncertainty.setZero();
  return result;
}

}  // namespace fast_livo_plane
