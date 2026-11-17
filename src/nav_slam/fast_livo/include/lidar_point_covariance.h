#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace fast_livo_lidar {

struct Noise {
  // Keep the legacy float multiplication for range noise.
  double range_variance;
  double angular_variance;
  Noise(float range_error, float angular_error_degrees)
      : range_variance(static_cast<float>(range_error * range_error)),
        // Match PCL's DEG2RAD macro used by the original implementation,
        // including its rounded conversion constant.
        angular_variance(std::pow(std::sin(angular_error_degrees * 0.017453293), 2)) {}
};

// The two angular variances are equal. For an orthonormal tangent basis N,
// [d]x N N' [d]x' = I - dd'. This removes constructing and normalizing that
// arbitrary basis for every point; the sensor error model is unchanged.
// The caller retains the existing zero-z convention before calling this.
inline Eigen::Matrix3d bodyCovariance(const Eigen::Vector3d &point, const Noise &noise) {
  const float range = std::sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
  const Eigen::Vector3d direction = point.normalized();
  const Eigen::Matrix3d radial = direction * direction.transpose();
  const double tangent_variance = static_cast<double>(range) * range * noise.angular_variance;
  return noise.range_variance * radial + tangent_variance * (Eigen::Matrix3d::Identity() - radial);
}

}  // namespace fast_livo_lidar
