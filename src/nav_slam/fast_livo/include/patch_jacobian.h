#pragma once

#include <Eigen/Core>

namespace fast_livo_visual {

// These transforms are constant for every pixel of one projected point.
// Factor them once per point/iteration; only the image gradient varies by pixel.
inline Eigen::Matrix<double, 2, 6> factorProjectionJacobian(
    const Eigen::Matrix<double, 2, 3> &projection,
    const Eigen::Matrix3d &point_hat,
    const Eigen::Matrix3d &rotation_derivative,
    const Eigen::Matrix3d &position_rotation_derivative,
    const Eigen::Matrix3d &position_translation_derivative) {
  Eigen::Matrix<double, 2, 6> result;
  result.leftCols<3>().noalias() =
      projection * (point_hat * rotation_derivative - position_rotation_derivative);
  result.rightCols<3>().noalias() = -projection * position_translation_derivative;
  return result;
}

}  // namespace fast_livo_visual
