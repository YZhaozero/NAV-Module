#include "lidar_point_covariance.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

static Eigen::Matrix3d legacy(const Eigen::Vector3d &point, float range_error, float angle_error) {
  const float range = std::sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]);
  const float range_variance = range_error * range_error;
  Eigen::Matrix2d angular;
  angular << std::pow(std::sin(angle_error * .017453293), 2), 0.,
      0., std::pow(std::sin(angle_error * .017453293), 2);
  Eigen::Vector3d direction(point); direction.normalize();
  Eigen::Matrix3d skew;
  skew << 0., -direction.z(), direction.y(), direction.z(), 0., -direction.x(), -direction.y(), direction.x(), 0.;
  Eigen::Vector3d tangent(1., 1., -(direction.x()+direction.y())/direction.z());
  tangent.normalize();
  Eigen::Vector3d second = tangent.cross(direction); second.normalize();
  Eigen::Matrix<double,3,2> basis;
  basis.col(0)=tangent; basis.col(1)=second;
  const Eigen::Matrix<double,3,2> a = range * skew * basis;
  return direction * range_variance * direction.transpose() + a * angular * a.transpose();
}

int main(int argc, char **) {
  std::mt19937 rng(429);
  std::uniform_real_distribution<double> uniform(-150.,150.);
  std::vector<Eigen::Vector3d> points;
  for (int i=0;i<50000;++i) points.emplace_back(uniform(rng),uniform(rng),uniform(rng));
  for (double scale : {1e-6, .01, 1., 100., 1000.}) {
    for (double z : {1e-14, -1e-14, .0001, -.001, 1., -1.}) {
      points.emplace_back(scale,scale,z);
      points.emplace_back(scale,-scale,z);
      points.emplace_back(0.,0.,z);
    }
  }
  double maximum_relative_error=0.;
  for (const auto &point : points) {
    for (const auto noise : {fast_livo_lidar::Noise(.02f,.05f),
                             fast_livo_lidar::Noise(.02f,0.f),
                             fast_livo_lidar::Noise(0.f,.05f)}) {
      const float range_error = noise.range_variance == 0. ? 0.f : .02f;
      const float angle_error = noise.angular_variance == 0. ? 0.f : .05f;
      const Eigen::Matrix3d expected=legacy(point,range_error,angle_error);
      const Eigen::Matrix3d actual=fast_livo_lidar::bodyCovariance(point,noise);
      const double relative=(actual-expected).norm()/std::max(1e-30,expected.norm());
      maximum_relative_error=std::max(maximum_relative_error,relative);
      if(!actual.allFinite() || relative>2e-12)
        throw std::runtime_error("Lidar point uncertainty model changed");
    }
  }
  // The fused point transform must retain the original intermediate PCL
  // float rounding, and may not substitute covariance zero-z geometry.
  const Eigen::Matrix3d extrinsic=Eigen::AngleAxisd(.41,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
  const Eigen::Vector3d translation(.023,-.014,.017);
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(-.74,Eigen::Vector3d(2.,-1.,1.).normalized()).toRotationMatrix();
  const Eigen::Vector3d position(121.3,-42.1,3.4);
  points.emplace_back(1.,2.,0.);
  points.emplace_back(0.,0.,0.);
  for (const auto &point : points) {
    const Eigen::Vector3d original=(rotation*(extrinsic*point+translation)+position);
    const float old_x=original.x(),old_y=original.y(),old_z=original.z();
    const Eigen::Vector3d prepared=extrinsic*point+translation;
    const Eigen::Vector3d fused=(rotation*prepared+position).cast<float>().cast<double>();
    if(fused.x()!=old_x || fused.y()!=old_y || fused.z()!=old_z)
      throw std::runtime_error("Fused lidar transform changed float rounding");
  }
  std::cout << points.size() << " points; max covariance relative error=" << maximum_relative_error << '\n';
  if(argc>1) {
    // Microbenchmark is optional; regression does not depend on host speed.
    points.resize(50000);
    volatile double checksum=0.;
    const fast_livo_lidar::Noise noise(.02f,.05f);
    const auto begin=std::chrono::steady_clock::now();
    for(int repeat=0;repeat<8;++repeat) for(const auto &point:points)
      checksum+=legacy(point,.02f,.05f)(0,0);
    const auto middle=std::chrono::steady_clock::now();
    for(int repeat=0;repeat<8;++repeat) for(const auto &point:points)
      checksum+=fast_livo_lidar::bodyCovariance(point,noise)(0,0);
    const auto end=std::chrono::steady_clock::now();
    std::cout << "legacy_s=" << std::chrono::duration<double>(middle-begin).count()
              << " optimized_s=" << std::chrono::duration<double>(end-middle).count()
              << " checksum=" << checksum << '\n';
  }
}
