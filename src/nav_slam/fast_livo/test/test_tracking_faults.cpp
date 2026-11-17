// Exercise mapper callbacks and the actual ROS publication/service boundaries.
#include "LIVMapper.h"
#include <rcl/publisher.h>
#include <pcl/io/pcd_io.h>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
volatile std::sig_atomic_t g_stop_requested = 0;
namespace {
std::mutex gate;
std::condition_variable changed;
bool entered=false, released=false;
std::atomic<int> poses{0}, malformed{0};
std::atomic<int> completed{0};
std::string scenario;
void require(bool yes, const char *why) { if (!yes) throw std::runtime_error(why); }
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t *pub, const void *data,
                                 rmw_publisher_allocation_t *) {
  const std::string topic=rcl_publisher_get_topic_name(pub);
  if (scenario=="pose" && topic=="/aft_mapped_to_init") {
    const int index=poses.fetch_add(1);
    if (index==0) {
      std::unique_lock<std::mutex> lock(gate); entered=true; changed.notify_all();
      changed.wait_for(lock, std::chrono::seconds(6), [] { return released; });
    }
    const auto &pose=*static_cast<const nav_msgs::msg::Odometry *>(data);
    if (std::abs(rclcpp::Time(pose.header.stamp).seconds()-(10.+index*.1))>1e-6 ||
        pose.pose.pose.position.x!=index || pose.child_frame_id.empty()) ++malformed;
    ++completed;
  }
  return RCL_RET_OK;
}
int main(int argc, char **argv) {
  scenario=argc>1 ? argv[1] : "pose";
  const char *arguments[]={"tracking_faults", "--ros-args", "--params-file",
      ROOT_DIR "config/mid360_live.yaml", "-p", "common.img_en:=0",
      "-p", "pcd_save.pcd_save_en:=false", "-p", "navigation.enabled:=true"};
  rclcpp::init(10, arguments);
  auto calibration=std::make_shared<rclcpp::Node>("parameter_blackboard");
  calibration->declare_parameter("cam_model","Pinhole");
  calibration->declare_parameter("cam_width",64); calibration->declare_parameter("cam_height",64);
  calibration->declare_parameter("scale",1.0);
  for (auto key:{"cam_fx","cam_fy"}) calibration->declare_parameter(key,40.);
  for (auto key:{"cam_cx","cam_cy"}) calibration->declare_parameter(key,32.);
  for (auto key:{"cam_d0","cam_d1","cam_d2","cam_d3"}) calibration->declare_parameter(key,0.);
  rclcpp::executors::SingleThreadedExecutor calibration_executor;
  calibration_executor.add_node(calibration);
  std::thread serving([&] { calibration_executor.spin(); });
  int result=0;
  try {
    auto owner=std::make_shared<rclcpp::Node>("tracking_fault_owner");
    LIVMapper mapper(owner,"tracking_fault_mapper");
    std::unique_ptr<vk::AbstractCamera> camera(mapper.vio_manager->cam);
    if (scenario=="pose") {
      mapper.pubOdomAftMapped=mapper.node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init",10);
      mapper.geoQuat.w=1.; mapper.geoQuat.x=mapper.geoQuat.y=mapper.geoQuat.z=0.;
      const auto begin=std::chrono::steady_clock::now();
      for (int i=0;i<101;++i) {
        mapper.LidarMeasures.last_lio_update_time=10.+i*.1; mapper._state.pos_end.x()=i;
        mapper.publish_odometry(mapper.pubOdomAftMapped);
        if (!i) {
          std::unique_lock<std::mutex> lock(gate);
          require(changed.wait_for(lock,std::chrono::seconds(2),[] { return entered; }),"publisher not entered");
        }
      }
      const double submit_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
      const bool still_blocked=poses==1;
      if (still_blocked) std::this_thread::sleep_for(std::chrono::milliseconds(5400));
      { std::lock_guard<std::mutex> lock(gate); released=true; } changed.notify_all();
      for (int i=0;i<500 && completed<101;++i) std::this_thread::sleep_for(std::chrono::milliseconds(10));
      std::cout<<"pose producer seconds="<<submit_s<<" delivered="<<poses<<" malformed="<<malformed<<std::endl;
      require(still_blocked && submit_s<.5,"DDS pose publish blocked estimator");
      require(completed==101 && malformed==0,"immutable ordered pose history lost");
    } else if (scenario=="transform") {
      mapper.extR=Eigen::AngleAxisd(.27,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
      mapper.extT=Eigen::Vector3d(.03,-.04,.05);
      const auto rotation=Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Eigen::Vector3d translation(-10.,20.,.7);
      PointCloudXYZI::Ptr input(new PointCloudXYZI()), output(new PointCloudXYZI());
      for (int n : {0, 17, 36001, 2}) {
        input->resize(n);
        for (int i=0;i<n;++i) {
          auto &p=input->points[i]; p.x=(i%157)*.17-12.; p.y=(i%61)*.13-3.; p.z=(i%23)*.07; p.intensity=i%255;
        }
        mapper.transformLidar(rotation,translation,input,output);
        require(output->size()==input->size(),"transform point count changed");
        for (int i=0;i<n;++i) {
          const auto &p=input->points[i]; const auto &q=output->points[i];
          const Eigen::Vector3d expected=rotation*(mapper.extR*Eigen::Vector3d(p.x,p.y,p.z)+mapper.extT)+translation;
          require(q.x==float(expected.x()) && q.y==float(expected.y()) && q.z==float(expected.z()) && q.intensity==p.intensity,
              "parallel transform changed geometry or order");
        }
      }
      std::cout<<"transform: empty, sparse, dense and scratch-reuse geometry matches serial reference"<<std::endl;
    } else {
      NavigationOutput::Cloud cloud;
      for (int i=0;i<40;++i) { pcl::PointXYZINormal p{}; p.x=i*.3; p.z=1.; cloud.push_back(p); }
      mapper.navigation_output_->publish(cloud,cloud,sec2Stamp(10.),"base_link","odom");
      mapper.imu_en=true; mapper.last_timestamp_lidar=10.; mapper.last_timestamp_imu=10.;
      mapper.LidarMeasures.last_lio_update_time=10.;
      mapper.ekf_finish_once=true;
      auto message=std::make_shared<sensor_msgs::msg::Imu>(); message->header.stamp=sec2Stamp(13.);
      mapper.imu_cbk(message); // Old implementation throws, taking SaveMap down.
      require(!mapper.sync_packages(mapper.LidarMeasures),"integrated unsupported gap");
      require(mapper.wait_reason_=="tracking_suspended","fault not explicit");
      for (int i=0;i<5000;++i) { message->header.stamp=sec2Stamp(13.+i*.005); mapper.imu_cbk(message); }
      require(mapper.imu_buffer.empty() && !mapper.ekf_finish_once,"paused input grew or propagation continued");
      require(mapper.LidarMeasures.last_lio_update_time==10.,"fabricated state after gap");
      char root[]="/tmp/tracking-fault-save-XXXXXX"; require(mkdtemp(root),"temporary directory");
      auto client=mapper.node->create_client<fast_livo::srv::SaveMap>("/fast_livo/save_map");
      require(client->wait_for_service(std::chrono::seconds(2)),"save service disappeared");
      auto request=std::make_shared<fast_livo::srv::SaveMap::Request>();
      request->save_path=std::string(root)+"/map.pcd"; request->leaf_size=.15;
      rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(mapper.node);
      auto future=client->async_send_request(request);
      require(executor.spin_until_future_complete(future,std::chrono::seconds(15))==rclcpp::FutureReturnCode::SUCCESS,"save timed out after gap");
      require(future.get()->success,"save failed after gap");
      pcl::PointCloud<pcl::PointXYZI> saved;
      require(pcl::io::loadPCDFile(request->save_path,saved)==0 && saved.size()==40,"map geometry lost after gap");
      std::filesystem::remove_all(root);
      std::cout<<"gap: source gap retained; service saved all 40 measured points; input bounded"<<std::endl;
    }
  } catch (const std::exception &e) { std::cerr<<e.what()<<std::endl; result=1; }
  { std::lock_guard<std::mutex> lock(gate); released=true; } changed.notify_all();
  calibration_executor.cancel(); serving.join(); rclcpp::shutdown(); return result;
}
