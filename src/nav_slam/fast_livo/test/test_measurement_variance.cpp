#include "measurement_variance.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

static void compare(const Eigen::Matrix3d &covariance,
                    const Eigen::Vector3d &normal,
                    const Eigen::Matrix3d &rotation) {
  const Eigen::Matrix3d rotated = rotation * covariance * rotation.transpose();
  const double expected = normal.transpose() * rotated * normal;
  const double actual = fast_livo_measurement::projectedVariance(
      covariance, normal, rotation.transpose());
  if (!std::isfinite(actual) || std::abs(actual-expected) > 1e-12 * std::max(1., std::abs(expected)))
    throw std::runtime_error("Projected measurement variance changed");
}

int main() {
  std::mt19937 rng(428);
  std::normal_distribution<double> random(0., 1.);
  for (int sample = 0; sample < 10000; ++sample) {
    Eigen::Matrix3d a;
    for (int i = 0; i < 9; ++i) a.data()[i] = random(rng);
    const Eigen::Vector3d normal(random(rng), random(rng), random(rng));
    const Eigen::Vector3d axis(random(rng), random(rng), random(rng));
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(random(rng), axis.normalized()).toRotationMatrix();
    compare(a * a.transpose() * .001, normal, rotation);
    // The identity is valid for singular and slightly asymmetric estimates too.
    compare(Eigen::Matrix3d::Zero(), normal, rotation);
    compare(normal * normal.transpose(), normal, rotation);
    compare(a, normal, rotation);
    compare(a, Eigen::Vector3d::Zero(), rotation);
  }
}
