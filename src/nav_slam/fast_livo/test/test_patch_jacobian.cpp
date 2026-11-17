#include "patch_jacobian.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

int main() {
  std::mt19937 rng(882811);
  std::uniform_real_distribution<double> value(-2., 2.);
  Eigen::Matrix<double, 6, 6> before_hessian = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> after_hessian = before_hessian;
  Eigen::Matrix<double, 6, 1> before_rhs = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> after_rhs = before_rhs;
  for (int point = 0; point < 1000; ++point) {
    Eigen::Matrix<double, 2, 3> projection;
    Eigen::Matrix3d point_hat, rotation, position_rotation, translation;
    for (int i = 0; i < projection.size(); ++i) projection.data()[i] = value(rng);
    for (auto *matrix : {&point_hat, &rotation, &position_rotation, &translation})
      for (int i = 0; i < matrix->size(); ++i) matrix->data()[i] = value(rng);
    const auto factored = fast_livo_visual::factorProjectionJacobian(
        projection, point_hat, rotation, position_rotation, translation);
    for (int pixel = 0; pixel < 64; ++pixel) {
      Eigen::Matrix<double, 1, 2> gradient;
      gradient << value(rng) * 128., value(rng) * 128.;
      // Independent original expression, including its intermediate products.
      const Eigen::RowVector3d phi = gradient * projection * point_hat;
      const Eigen::RowVector3d position = -gradient * projection;
      Eigen::Matrix<double, 1, 6> before;
      before << (phi * rotation + position * position_rotation).eval(),
                (position * translation).eval();
      const Eigen::Matrix<double, 1, 6> after = gradient * factored;
      if (!after.allFinite() || (before - after).norm() > 1e-12 * (1. + before.norm()))
        throw std::runtime_error("Factored photometric Jacobian differs");
      const double residual = value(rng) * 128.;
      before_hessian.noalias() += before.transpose() * before;
      after_hessian.noalias() += after.transpose() * after;
      before_rhs.noalias() += before.transpose() * residual;
      after_rhs.noalias() += after.transpose() * residual;
    }
  }
  if ((before_hessian - after_hessian).norm() > 1e-12 * before_hessian.norm() ||
      (before_rhs - after_rhs).norm() > 1e-12 * before_rhs.norm())
    throw std::runtime_error("Factored Jacobian changed normal equations");
}
