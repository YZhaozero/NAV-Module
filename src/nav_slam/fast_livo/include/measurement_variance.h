#pragma once
#include <Eigen/Core>

namespace fast_livo_measurement {

// n^T (R C R^T) n = (R^T n)^T C (R^T n). Only the scalar variance
// is consumed by the residual weight, so no rotated 3x3 temporary is needed.
inline double projectedVariance(const Eigen::Matrix3d &covariance,
                                const Eigen::Vector3d &normal,
                                const Eigen::Matrix3d &world_to_sensor) {
  const Eigen::Vector3d local_normal = world_to_sensor * normal;
  return local_normal.dot(covariance * local_normal);
}

}  // namespace fast_livo_measurement
