#include "plane_covariance.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Cov = Eigen::Matrix<double, 6, 6>;
struct Point { Vec point_w; Mat var; };
using Points = std::vector<Point>;
using Fit = fast_livo_plane::Fit;

static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

// Original complex eigensolver and per-point six-by-three Jacobian. Guard the
// known out-of-range middle eigenvector, so this reference never invokes UB.
static Fit reference(const Points &points) {
  Fit result;
  for (const auto &p : points) {
    result.center += p.point_w;
    result.covariance += p.point_w * p.point_w.transpose();
  }
  const double count = points.size();
  result.center /= count;
  result.covariance = result.covariance / count - result.center * result.center.transpose();
  Eigen::EigenSolver<Mat> solver(result.covariance);
  const Vec values = solver.eigenvalues().real();
  Eigen::Index low, high;
  values.minCoeff(&low); values.maxCoeff(&high);
  const int middle = 3 - low - high;
  if (low == high || middle < 0 || middle > 2 || values[low] >= .01) return result;
  result.eigenvalues << values[low], values[middle], values[high];
  result.eigenvectors.col(0) = solver.eigenvectors().real().col(low);
  result.eigenvectors.col(1) = solver.eigenvectors().real().col(middle);
  result.eigenvectors.col(2) = solver.eigenvectors().real().col(high);
  const Mat center_jacobian = Mat::Identity() / count;
  for (const auto &point : points) {
    Mat derivative = Mat::Zero();
    for (int axis = 1; axis < 3; ++axis) {
      derivative.row(axis) = (point.point_w - result.center).transpose() /
          (count * (result.eigenvalues[0] - result.eigenvalues[axis])) *
          (result.eigenvectors.col(axis) * result.eigenvectors.col(0).transpose() +
           result.eigenvectors.col(0) * result.eigenvectors.col(axis).transpose());
    }
    Eigen::Matrix<double, 6, 3> jacobian;
    jacobian.topRows<3>() = result.eigenvectors * derivative;
    jacobian.bottomRows<3>() = center_jacobian;
    result.uncertainty += jacobian * point.var * jacobian.transpose();
  }
  result.valid = result.uncertainty.allFinite();
  return result;
}

struct Comparison {
  size_t count=0, planes=0;
  double normal=0., covariance=0., residual_variance=0.;
  void check(const Points &points) {
    ++count;
    const auto before=reference(points), after=fast_livo_plane::estimate(points,.01);
    require(before.valid == after.valid, "nondegenerate plane classification changed");
    if (!before.valid) return;
    ++planes;
    const double sign=before.eigenvectors.col(0).dot(after.eigenvectors.col(0)) >= 0. ? 1. : -1.;
    Cov flip=Cov::Identity(); flip.topLeftCorner<3,3>() *= sign;
    normal=std::max(normal,(before.eigenvectors.col(0)-sign*after.eigenvectors.col(0)).norm());
    covariance=std::max(covariance,(before.uncertainty-flip*after.uncertainty*flip).norm() /
                       std::max(1e-30,before.uncertainty.norm()));
    for (const Vec &delta : {Vec(.2,.4,.1),Vec(-1.,.3,2.)}) {
      Eigen::Matrix<double,1,6> old_j,new_j;
      old_j << delta.transpose(), -before.eigenvectors.col(0).transpose();
      new_j << delta.transpose(), -after.eigenvectors.col(0).transpose();
      const double old_var=(old_j*before.uncertainty*old_j.transpose())(0,0);
      const double new_var=(new_j*after.uncertainty*new_j.transpose())(0,0);
      residual_variance=std::max(residual_variance,std::abs(old_var-new_var)/std::max(1e-30,std::abs(old_var)));
    }
    require(normal<1e-7 && covariance<1e-6 && residual_variance<1e-6,
            "normal, covariance, or residual variance changed");
  }
  void print(const char *label) const {
    std::cout << label << " groups=" << count << " planes=" << planes
              << " normal_error=" << normal << " covariance_relative_error=" << covariance
              << " residual_variance_relative_error=" << residual_variance << '\n';
  }
};

int main(int argc,char **argv) {
  std::cout.precision(12);
  std::mt19937 rng(20261028);
  std::normal_distribution<double> gaussian(0.,1.);
  std::uniform_real_distribution<double> uniform(-1.,1.);
  std::vector<Points> dataset;
  Comparison synthetic;
  for (int group=0;group<1000;++group) {
    const Vec axis=Vec(gaussian(rng),gaussian(rng),gaussian(rng)).normalized();
    const Mat rotation=Eigen::AngleAxisd(uniform(rng)*3.,axis).toRotationMatrix();
    const Vec center=Vec(uniform(rng),uniform(rng),uniform(rng))*(group%3==0?100.:5.);
    Points points;
    for(int i=0;i<6+group%45;++i) {
      Mat covariance=rotation*Vec(.001,.002,.003).asDiagonal()*rotation.transpose();
      if(i%10==0) covariance(0,1)+=1e-9; // Preserve mildly asymmetric state covariance too.
      points.push_back({rotation*Vec(gaussian(rng)*.45,gaussian(rng)*.21,gaussian(rng)*.0002)+center,covariance});
    }
    synthetic.check(points);
    dataset.push_back(std::move(points));
  }
  synthetic.print("synthetic");

  Points degenerate;
  for(int i=0;i<8;++i) degenerate.push_back({Vec(i,0,0),Mat::Identity()*.001});
  const auto line=fast_livo_plane::estimate(degenerate,.01);
  require(!line.valid && line.uncertainty.allFinite(),"collinear points accepted or produced NaN");
  for(auto &point:degenerate) point.point_w.setZero();
  const auto duplicate=fast_livo_plane::estimate(degenerate,.01);
  require(!duplicate.valid && duplicate.uncertainty.allFinite(),"duplicate points accepted or produced NaN");
  require(!fast_livo_plane::estimate(Points{},.01).valid,"empty group accepted");
  Points circle;
  for(int i=0;i<4;++i) circle.push_back({Vec((i&1)?1.:-1.,(i&2)?1.:-1.,0),Mat::Identity()*.001});
  require(fast_livo_plane::estimate(circle,.01).valid,"equal tangent eigenvalues rejected");
  for(int axis=0;axis<3;++axis) {
    Vec point=Vec::Zero(); point[axis]=1.;
    degenerate.push_back({point,Mat::Identity()*.001});
    degenerate.push_back({-point,Mat::Identity()*.001});
  }
  require(!fast_livo_plane::estimate(degenerate,1.).valid,"isotropic covariance defines no normal");
  circle[0].var(0,0)=std::numeric_limits<double>::infinity();
  const auto invalid=fast_livo_plane::estimate(circle,.01);
  require(!invalid.valid && invalid.uncertainty.allFinite(),"nonfinite measurement covariance escaped");
  std::cout << "degenerate_cases=6 passed\n";

  if(argc>1) {
    std::ifstream input(argv[1]);
    require(bool(input),"cannot open real PCD groups");
    dataset.clear();
    Comparison measured;
    int count=0;
    while(input>>count) {
      require(count>=6 && count<=50,"invalid voxel point count");
      Points points(count);
      for(auto &point:points) {
        input>>point.point_w.x()>>point.point_w.y()>>point.point_w.z();
        point.var=Mat::Identity()*.001;
      }
      require(bool(input),"truncated PCD group");
      measured.check(points);
      dataset.push_back(std::move(points));
    }
    measured.print("archived_pcd");
  }
  if(argc>2 && std::string(argv[2])=="--bench") {
    volatile double checksum=0.;
    for(int trial=0;trial<3;++trial) {
      const auto run=[&](bool optimized) {
        const auto start=std::chrono::steady_clock::now();
        for(int repeat=0;repeat<100;++repeat) for(const auto &points:dataset)
          checksum+=(optimized?fast_livo_plane::estimate(points,.01):reference(points)).uncertainty(0,0);
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      };
      const double before=run(false),after=run(true);
      std::cout << "trial=" << trial << " reference_s=" << before << " production_s=" << after
                << " speedup=" << before/after << '\n';
    }
    std::cout << "checksum=" << checksum << '\n';
  }
}
