#include "IMU_Processing.h"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Observe the pose history without changing the production class definition,
// layout or algorithm. All initialization/propagation uses public Process2().
template <typename Tag, typename Tag::type Member>
struct ImuDeskewProbe { friend typename Tag::type field(Tag) { return Member; } };
struct ImuDeskewPoses {
  using type = std::vector<Pose6D> ImuProcess::*;
  friend type field(ImuDeskewPoses);
};
template struct ImuDeskewProbe<ImuDeskewPoses, &ImuProcess::IMUpose>;

namespace {
const std::vector<Pose6D> &poses(ImuProcess &processor) {
  return processor.*field(ImuDeskewPoses{});
}
void require(bool okay, const std::string &message) {
  if (!okay) throw std::runtime_error(message);
}
sensor_msgs::msg::Imu::SharedPtr sample(double time, double angular_velocity) {
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->header.stamp = sec2Stamp(time);
  imu->linear_acceleration.z = 1.;
  imu->angular_velocity.z = angular_velocity;
  return imu;
}
LidarMeasureGroup interval(double start, double duration, double imu_period,
                           EKF_STATE kind, const std::vector<float> &offsets,
                           double angular_velocity = 1.) {
  LidarMeasureGroup group;
  group.lio_vio_flg = kind;
  group.last_lio_update_time = start;
  group.lidar_frame_beg_time = start;
  MeasureGroup measure;
  measure.lio_time = measure.vio_time = start + duration;
  const int count = static_cast<int>(std::llround(duration / imu_period));
  for (int i = 1; i <= count; ++i)
    measure.imu.push_back(sample(start + i * imu_period, angular_velocity));
  group.measures.push_back(measure);
  for (float offset : offsets) {
    PointType point{};
    point.x = 10.; point.curvature = offset;
    group.pcl_proc_cur->push_back(point);
  }
  return group;
}
struct Fixture {
  ImuProcess processor;
  StatesGroup state;
  PointCloudXYZI::Ptr output{new PointCloudXYZI};
  Fixture() {
    processor.first_lidar_time = .9;
    processor.set_imu_init_frame_num(10);
    auto init = interval(.9, .1, .005, LIO, {}, 0.);
    processor.Process2(init, state, output);
    require(!processor.imu_need_init, "production IMU initialization failed");
    require(std::abs(processor.IMU_mean_acc_norm - 1.) < 1e-12, "unexpected acceleration scale");
    processor.ResumeAfterGap(sample(1., 1.));
    // Establish the last measured angular rate via normal public propagation.
    auto warmup = interval(1., .1, .005, LIO, {2.f, 20.f, 50.f, 99.f});
    processor.Process2(warmup, state, output);
    require(poses(processor).empty(), "warmup retained poses");
  }
};
void sameState(const StatesGroup &a, const StatesGroup &b) {
  const double error = (a.rot_end - b.rot_end).norm() +
      (a.pos_end - b.pos_end).norm() + (a.vel_end - b.vel_end).norm() +
      (a.bias_a - b.bias_a).norm() + (a.bias_g - b.bias_g).norm() +
      (a.gravity - b.gravity).norm() + std::abs(a.inv_expo_time - b.inv_expo_time) +
      (a.cov - b.cov).norm();
  require(error < 1e-12, "deskew changed propagated state or covariance");
}
void analyticCloud(const PointCloudXYZI &cloud, const std::vector<float> &offsets,
                   double duration) {
  require(cloud.size() == offsets.size(), "deskew lost or duplicated points");
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    const auto &point = cloud[i];
    require(point.curvature == offsets[i], "deskew changed a point timestamp");
    const double angle = -(duration - double(offsets[i]) / 1000.);
    const double error = std::hypot(point.x - 10. * std::cos(angle),
                                    point.y - 10. * std::sin(angle));
    require(error < 2e-6 && std::abs(point.z) < 1e-7,
            "point was skipped, repeatedly compensated, or used another interval's IMU poses");
  }
}
struct Step { EKF_STATE kind; bool nonempty; double duration; };
void sequence(const std::vector<Step> &steps, float first_offset) {
  Fixture actual, propagation_only;
  double time = 1.1;
  for (const auto &step : steps) {
    const std::vector<float> offsets = step.nonempty
        ? std::vector<float>{first_offset, 20.f, 50.f, 99.f} : std::vector<float>{};
    auto group = interval(time, step.duration, .005, step.kind, offsets);
    auto reference = interval(time, step.duration, .005, step.kind, {});
    actual.processor.Process2(group, actual.state, actual.output);
    propagation_only.processor.Process2(reference, propagation_only.state, propagation_only.output);
    sameState(actual.state, propagation_only.state);
    require(std::abs(group.last_lio_update_time - reference.last_lio_update_time) < 1e-12,
            "deskew changed the propagation boundary");
    if (step.kind == LIO && step.nonempty) {
      analyticCloud(*actual.output, offsets, step.duration);
      require(poses(actual.processor).empty(), "completed scan retained IMU poses");
    } else {
      require(actual.output->empty(), "empty LIO/VIO returned an old point cloud");
      if (step.kind == LIO) {
        require(poses(actual.processor).size() == 21, "empty LIO accumulated earlier pose intervals");
        double previous = -1.;
        for (const auto &pose : poses(actual.processor)) {
          require(pose.offset_time >= previous, "pose offsets went backwards in a new LIO interval");
          previous = pose.offset_time;
        }
      }
    }
    time += step.duration;
  }
}
void allSequences() {
  for (float first : {0.f, 2.f, 12.f}) {
    sequence({{LIO,true,.1},{LIO,true,.1},{LIO,true,.1}}, first);
    sequence({{LIO,false,.1},{LIO,true,.1},{LIO,true,.1}}, first);
    sequence({{LIO,false,.1},{LIO,false,.1},{LIO,false,.1},{LIO,true,.1}}, first);
    sequence({{LIO,true,.1},{VIO,false,0.},{LIO,true,.1},{VIO,false,0.},{LIO,true,.1}}, first);
    sequence({{LIO,false,.1},{VIO,false,0.},{LIO,true,.1}}, first);
    // Robustness of Process2 when VIO advances time. Each following scan is
    // explicitly rebased to that VIO endpoint, independent of the synchronizer.
    sequence({{LIO,false,.1},{VIO,false,.02},{LIO,true,.1}}, first);
    sequence({{LIO,true,.1},{VIO,false,.02},{LIO,true,.1}}, first);
    sequence({{LIO,false,.1},{VIO,false,0.},{LIO,false,.1},{VIO,false,.02},{LIO,true,.1}}, first);
  }
}
void exactBoundaries() {
  const std::vector<std::vector<float>> cases = {
      {0.f,62.5f,125.f,187.5f,250.f}, {62.5f,125.f,187.5f,250.f},
      {0.f}, {12.f}, {125.f}, {250.f},
      {0.f,0.f,0.f,62.5f,125.f,125.f,250.f}, {125.f,125.f,125.f}};
  for (const auto &offsets : cases) {
    Fixture actual, propagation_only;
    actual.processor.ResumeAfterGap(sample(2., 1.));
    propagation_only.processor.ResumeAfterGap(sample(2., 1.));
    // These binary-exact times really equal IMUpose offsets; decimal rounding
    // cannot accidentally move the points to the other side of the comparison.
    auto group = interval(2., .25, .0625, LIO, offsets);
    auto reference = interval(2., .25, .0625, LIO, {});
    actual.processor.Process2(group, actual.state, actual.output);
    propagation_only.processor.Process2(reference, propagation_only.state, propagation_only.output);
    sameState(actual.state, propagation_only.state);
    analyticCloud(*actual.output, offsets, .25);
  }
}
} // namespace

namespace variable_motion {
constexpr double start = 2.;
constexpr double step = .0625;
constexpr double duration = .25;
using Samples = std::array<double, 5>;
const Samples zero{{0.,0.,0.,0.,0.}};
const Samples gyro{{0.,1.,3.,-2.,4.}};
const Samples accel{{0.,.1,-.05,.2,-.1}};
const V3D input(10.,2.,-1.);
const V3D extrinsic_translation(.4,-.2,.3);
M3D rz(double angle) {
  M3D result;
  result << std::cos(angle),-std::sin(angle),0.,
            std::sin(angle),std::cos(angle),0., 0.,0.,1.;
  return result;
}
M3D extrinsicRotation() {
  M3D result;
  const double c=std::cos(.37), s=std::sin(.37);
  result << c,0.,s, 0.,1.,0., -s,0.,c;
  return rz(-.23)*result;
}
sensor_msgs::msg::Imu::SharedPtr imu(double time, double w, double a) {
  auto result=sample(time,w);
  result->linear_acceleration.x=a;
  return result;
}
LidarMeasureGroup group(const Samples &w, const Samples &a,
                        const std::vector<float> &offsets) {
  LidarMeasureGroup result;
  result.lio_vio_flg=LIO;
  result.last_lio_update_time=result.lidar_frame_beg_time=start;
  MeasureGroup measurement;
  measurement.lio_time=measurement.vio_time=start+duration;
  for (int i=1;i<5;++i)
    measurement.imu.push_back(imu(start+i*step,w[i],a[i]));
  result.measures.push_back(measurement);
  for (float offset:offsets) {
    PointType point{};
    point.x=input.x(); point.y=input.y(); point.z=input.z();
    point.curvature=offset;
    result.pcl_proc_cur->push_back(point);
  }
  return result;
}
void setup(Fixture &fixture, const Samples &w, const Samples &a) {
  fixture.processor.set_extrinsic(extrinsic_translation,extrinsicRotation());
  // Set the cached previous interval's drive to this trajectory's initial
  // drive through public propagation, isolating varying-rate errors from the
  // original constant-rotation fixture's last rate.
  fixture.processor.ResumeAfterGap(imu(1.9,w[0],a[0]));
  auto warmup=interval(1.9,.1,.005,LIO,{2.f,20.f,50.f,99.f},w[0]);
  for (auto &measurement:warmup.measures)
    for (auto &sample:measurement.imu)
      sample=imu(stamp2Sec(sample->header.stamp),w[0],a[0]);
  fixture.processor.Process2(warmup,fixture.state,fixture.output);
  fixture.processor.ResumeAfterGap(imu(start,w[0],a[0]));
}
V3D point(const PointType &p) { return V3D(p.x,p.y,p.z); }
double endpoint(const std::string &name, const Samples &w, const Samples &a) {
  Fixture actual, control;
  setup(actual,w,a); setup(control,w,a);
  auto measured=group(w,a,{250.f});
  auto empty=group(w,a,{});
  actual.processor.Process2(measured,actual.state,actual.output);
  control.processor.Process2(empty,control.state,control.output);
  sameState(actual.state,control.state);
  require(actual.output->size()==1,"endpoint point count changed");
  const double error=(point(actual.output->front())-input).norm();
  std::cout<<"endpoint "<<name<<" identity_error_m="<<error
           <<" actual="<<point(actual.output->front()).transpose()<<'\n';
  return error;
}
// Independent closed-form trajectory: each interval's rate/acceleration is
// constant. Endpoint sample values encode its arithmetic mean, as used by
// the production propagator. Pure Z rotation and pure translation are exact
// for this model; neither oracle reads pose history nor calls Exp/deskew.
double angleAt(double time, const Samples &w) {
  double angle=0.;
  for(int k=0;k<4;++k) {
    const double overlap=std::max(0.,std::min(step,time-k*step));
    angle+=.5*(w[k]+w[k+1])*overlap;
  }
  return angle;
}
V3D positionAt(double time, const StatesGroup &initial, const Samples &a) {
  V3D position=initial.pos_end+initial.vel_end*time;
  for(int k=0;k<4;++k) {
    const double left=k*step;
    const double overlap=std::max(0.,std::min(step,time-left));
    const V3D acceleration=initial.rot_end*V3D(.5*(a[k]+a[k+1])*G_m_s2,0.,G_m_s2)+initial.gravity;
    // Analytic convolution of a rectangular acceleration pulse with (t-s).
    position+=acceleration*((time-left)*overlap-.5*overlap*overlap);
  }
  return position;
}
double intermediate(const std::string &name, bool rotation) {
  const Samples &w=rotation?gyro:zero;
  const Samples &a=rotation?zero:accel;
  Fixture actual,control;
  setup(actual,w,a); setup(control,w,a);
  const StatesGroup initial=actual.state;
  const std::vector<float> offsets{0.f,0.f,15.625f,62.5f,93.75f,125.f,125.f,187.5f,234.375f,250.f};
  auto measured=group(w,a,offsets);
  auto empty=group(w,a,{});
  actual.processor.Process2(measured,actual.state,actual.output);
  control.processor.Process2(empty,control.state,control.output);
  sameState(actual.state,control.state);
  const M3D extrinsic=extrinsicRotation();
  const M3D rotation_end=initial.rot_end*rz(angleAt(duration,w));
  const V3D position_end=positionAt(duration,initial,a);
  require((actual.state.rot_end-rotation_end).norm()<1e-12,"forward rotation disagrees with physical oracle");
  require((actual.state.pos_end-position_end).norm()<1e-12,"forward position disagrees with physical oracle");
  double maximum=0.;
  require(actual.output->size()==offsets.size(),"intermediate point count changed");
  for(std::size_t i=0;i<offsets.size();++i) {
    const double t=double(offsets[i])/1000.;
    const M3D rotation_at_measurement=initial.rot_end*rz(angleAt(t,w));
    const V3D world_point=rotation_at_measurement*(extrinsic*input+extrinsic_translation)+positionAt(t,initial,a);
    const V3D expected=extrinsic.transpose()*(rotation_end.transpose()*(world_point-position_end)-extrinsic_translation);
    maximum=std::max(maximum,(point((*actual.output)[i])-expected).norm());
    require((*actual.output)[i].curvature==offsets[i],"point stamp changed");
  }
  std::cout<<"intermediate "<<name<<" max_error_m="<<maximum<<" points="<<offsets.size()<<'\n';
  return maximum;
}
} // namespace variable_motion
namespace variable_motion {
void allCases() {
  std::cout<<std::setprecision(15);
  double maximum=0.;
  maximum=std::max(maximum,endpoint("varying_gyro",gyro,zero));
  maximum=std::max(maximum,endpoint("varying_acceleration",zero,accel));
  maximum=std::max(maximum,endpoint("varying_both",gyro,accel));
  maximum=std::max(maximum,intermediate("piecewise_constant_z_rate",true));
  maximum=std::max(maximum,intermediate("piecewise_constant_body_x_acceleration",false));
  require(maximum<2e-6,"deskew disagrees with independent endpoint/physical oracle");
}
} // namespace variable_motion

int main() {
  try {
    allSequences();
    exactBoundaries();
    variable_motion::allCases();
    std::cout << "PASS: production IMU deskew, 24 LIO/VIO sequences, 8 exact-boundary/single-point/duplicate-time cases, 3 varying-motion endpoints and 2 analytic trajectories\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
