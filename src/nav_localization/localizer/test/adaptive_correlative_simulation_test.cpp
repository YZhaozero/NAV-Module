#include "../src/localizers/correlative_search.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <random>
#include <string>

using Cloud = CorrelativeSearch::Cloud;
static void add(Cloud& c, float x, float y, float z) {
  pcl::PointXYZI p; p.x=x; p.y=y; p.z=z; c.push_back(p);
}
// Analytic surfaces are sampled independently for map and sensor. No map
// point indices or true pose are passed to the matcher.
static Cloud room(float step, float ceiling, float roof_fraction=1.f, float slope=0.f) {
  Cloud c;
  auto put=[&](float x,float y,float z){add(c,x,y,z+slope*x);};
  for(float x=-9;x<9;x+=step) for(float y=-6;y<7;y+=step) {
    put(x,y,0);
    if(ceiling>0 && x < -9+18*roof_fraction) put(x,y,ceiling);
  }
  const float wall=ceiling>0?ceiling:9.f;
  for(float x=-8;x<8;x+=step) for(float z=.1;z<wall;z+=step) put(x,-5,z);
  for(float y=-5;y<6;y+=step) for(float z=.1;z<wall*.8f;z+=step) put(-8,y,z);
  for(float y=-5;y<1.7f;y+=step) for(float z=.1;z<wall*.6f;z+=step) put(7.4f,y,z);
  for(auto box:std::vector<Eigen::Vector3f>{{-2,2,1.1f},{3,-1,1.8f},{4,4,2.1f}}) {
    for(float u=0;u<1.6f;u+=step) for(float z=.1;z<box.z();z+=step) {
      put(box.x()+u,box.y(),z);put(box.x(),box.y()+u*.6f,z);
    }
    for(float u=0;u<1.6f;u+=step) for(float v=0;v<.96f;v+=step) put(box.x()+u,box.y()+v,box.z());
  }
  if(ceiling==0) for(float z=0;z<18;z+=step) for(float a=0;a<6.28f;a+=.4f)
    put(2+.3f*std::cos(a),5+.3f*std::sin(a),z);
  return c;
}
static Cloud moved(const Cloud& c,const Eigen::Matrix4f& pose,float noise=0,float keep=1) {
  std::mt19937 rng(1129);std::normal_distribution<float> n(0,noise);
  std::uniform_real_distribution<float> u(0,1);
  Cloud q;
  const Eigen::Matrix3f inv=pose.block<3,3>(0,0).transpose();
  for(const auto& p:c) if(u(rng)<=keep) {
    const Eigen::Vector3f v=inv*(p.getVector3fMap()-pose.block<3,1>(0,3));
    add(q,v.x()+n(rng),v.y()+n(rng),v.z()+n(rng));
  }
  return q;
}
static Cloud translated(const Cloud& c,const Eigen::Vector3f& t) {
  Cloud q;for(const auto& p:c) add(q,p.x+t.x(),p.y+t.y(),p.z+t.z());return q;
}
static Cloud corridor(float step,float length) {
  Cloud c;
  for(float x=-length;x<length;x+=step) {
    for(float y=-2;y<2;y+=step){add(c,x,y,0);add(c,x,y,3.4f);}
    for(float z=.1f;z<3.4f;z+=step){add(c,x,-2,z);add(c,x,2,z);}
  }
  for(float x=-20;x<=20;x+=4) if(std::abs(x)<length)
    for(float y=1.4f;y<2;y+=step)for(float z=0;z<2.3f;z+=step)add(c,x,y,z);
  return c;
}
static int failures=0,cases=0;
static void check(const std::string& name,const Cloud& map,const Cloud& observed,
                  const Eigen::Matrix4f& truth,const Eigen::Matrix3f& level,
                  bool identifiable,float noise=0,float keep=1,CorrelativeSearch* reusable=nullptr) {
  const auto begin=std::chrono::steady_clock::now();
  CorrelativeSearch local;
  if(!reusable)local.build(map);
  auto& search=reusable?*reusable:local;
  const auto r=search.searchWithDiagnostics(moved(observed,truth,noise,keep),level);
  float translation=1e6f,angle=1e6f;
  if(!r.proposals.empty()) {
    translation=(r.proposals[0].pose.block<3,1>(0,3)-truth.block<3,1>(0,3)).norm();
    angle=Eigen::AngleAxisf(r.proposals[0].pose.block<3,3>(0,0)*truth.block<3,3>(0,0).transpose()).angle()*180/M_PI;
  }
  const bool pass=identifiable ? r.unique && translation<.85f && angle<4.f : !r.unique;
  ++cases;if(!pass)++failures;
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  std::cout<<"{\"case\":\""<<name<<"\",\"pass\":"<<(pass?"true":"false")
    <<",\"unique\":"<<(r.unique?"true":"false")<<",\"gap\":"<<r.gap
    <<",\"translation_m\":"<<translation<<",\"angle_deg\":"<<angle
    <<",\"layers\":"<<r.layer_count<<",\"heights\":"<<r.height_hypotheses
    <<",\"fine\":"<<(r.refined_grid?"true":"false")<<",\"seconds\":"<<seconds<<"}"<<std::endl;
}

int main() {
  cv::setNumThreads(1); // Only the standalone test controls OpenCV parallelism.
  const Eigen::Matrix3f identity=Eigen::Matrix3f::Identity();
  Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  pose.block<3,3>(0,0)=Eigen::AngleAxisf(64.1f*M_PI/180,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  pose.block<3,1>(0,3)=Eigen::Vector3f(2.3f,-1.7f,1.1f);
  for(float h:{2.2f,3.4f,5.2f,8.f,0.f})
    check("ceiling_"+std::to_string(h),room(.2f,h),room(.23f,h),pose,identity,true);
  check("partial_canopy",room(.2f,3.4f,.18f),room(.23f,3.4f,.18f),pose,identity,true);
  check("noise_and_missing_returns",room(.2f,3.4f),room(.23f,3.4f),pose,identity,true,.025f,.6f);
  Cloud partial;
  for(const auto& p:room(.23f,3.4f))if(p.x<5 && p.y<3)partial.push_back(p);
  check("partial_field_of_view",room(.2f,3.4f),partial,pose,identity,true,.02f,.7f);
  for(float yaw:{-117.8f,12.2f,174.6f}){
    auto rotated=pose;rotated.block<3,3>(0,0)=Eigen::AngleAxisf(yaw*M_PI/180,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    check("yaw_"+std::to_string(yaw),room(.2f,3.4f),room(.23f,3.4f),rotated,identity,true);
  }
  const auto level=(Eigen::AngleAxisf(.12f,Eigen::Vector3f::UnitX()) *
                    Eigen::AngleAxisf(-.08f,Eigen::Vector3f::UnitY())).toRotationMatrix();
  auto tilted=pose;tilted.block<3,3>(0,0)=pose.block<3,3>(0,0)*level;
  check("known_gravity_tilt",room(.2f,3.4f),room(.23f,3.4f),tilted,level,true);
  auto higher=pose;higher(2,3)+=.8f;
  check("sensor_height_changed",room(.2f,3.4f),room(.23f,3.4f),higher,identity,true);
  const Eigen::Vector3f shift(0,0,-4.5f);auto shifted=pose;shifted.block<3,1>(0,3)+=shift;
  check("map_height_origin_changed",translated(room(.2f,3.4f),shift),translated(room(.23f,3.4f),shift),shifted,identity,true);
  check("ramp",room(.2f,3.4f,1.f,.08f),room(.23f,3.4f,1.f,.08f),pose,identity,true);
  // Roof visibility changes without changing the map, representing a doorway
  // or sensor occlusion. These are visibility simulations, not ray-traced walks.
  for(float fraction:{.5f,.1f,0.f})
    check("roof_visibility_"+std::to_string(fraction),room(.2f,3.4f),room(.23f,3.4f,fraction),pose,identity,true);
  Cloud repeated=room(.2f,3.4f);
  repeated+=translated(repeated,Eigen::Vector3f(25,0,0));
  check("identical_rooms",repeated,room(.23f,3.4f),pose,identity,false);
  Cloud levels=room(.2f,3.4f);
  levels+=translated(levels,Eigen::Vector3f(0,0,4.5f));
  check("identical_storeys",levels,room(.23f,3.4f),pose,identity,false);
  Cloud platforms=room(.2f,3.4f);
  platforms+=translated(platforms,Eigen::Vector3f(0,0,.8f));
  check("overlapping_close_height_structures",platforms,room(.23f,3.4f),pose,identity,false);
  check("repetitive_corridor",corridor(.2f,24),corridor(.23f,6),pose,identity,false);
  Cloud flat;for(float x=-9;x<9;x+=.2f)for(float y=-6;y<7;y+=.2f)add(flat,x,y,0);
  check("featureless_ground",flat,flat,pose,identity,false);
  Cloud invalid=room(.2f,3.4f);
  add(invalid,NAN,0,0);add(invalid,0,INFINITY,0);
  check("nonfinite_map_points",invalid,room(.23f,3.4f),pose,identity,true);
  CorrelativeSearch sequence;const auto sequence_map=room(.2f,3.4f);sequence.build(sequence_map);
  int frame=0;
  for(float roof:{1.f,.5f,.1f,0.f,.5f,1.f}){
    auto walking=pose;walking(0,3)+=.2f*frame;
    check("visibility_sequence_"+std::to_string(frame++),sequence_map,room(.23f,3.4f,roof),walking,identity,true,.01f,.9f,&sequence);
  }
  const Eigen::Vector3f replacement(12,7,-.5f);
  auto replacement_pose=pose;replacement_pose.block<3,1>(0,3)+=replacement;
  sequence.build(translated(sequence_map,replacement));
  check("map_replaced_cache_reset",translated(sequence_map,replacement),translated(room(.23f,3.4f),replacement),replacement_pose,identity,true,0,1,&sequence);
  CorrelativeSearch empty;
  const bool cleared=empty.search(Cloud{},identity).empty();
  ++cases;if(!cleared)++failures;
  std::cout<<"{\"cases\":"<<cases<<",\"failures\":"<<failures<<"}"<<std::endl;
  return failures?1:0;
}
