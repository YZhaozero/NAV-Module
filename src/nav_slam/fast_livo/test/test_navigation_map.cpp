#include "navigation_output.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <thread>
#include <limits>
static void check(bool ok) { if(!ok)throw std::runtime_error("Navigation map integration failed"); }
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  char root[]="/tmp/nav-map-test-XXXXXX";check(mkdtemp(root));
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("navigation.enabled",true),
    rclcpp::Parameter("navigation.map_voxel_size",.15),
    rclcpp::Parameter("navigation.max_map_voxels",20),
    rclcpp::Parameter("navigation.preview_max_points",40),
    rclcpp::Parameter("navigation.map_spool_directory",std::string(root))});
  auto node=std::make_shared<rclcpp::Node>("navigation_map_integration",options);
  node->declare_parameter("navigation.body_frame","base_link");
  for(auto key:{"extrin_calib.extrinsic_R","extrin_calib.Rcl"})
    node->declare_parameter(key,std::vector<double>{1,0,0,0,1,0,0,0,1});
  for(auto key:{"extrin_calib.extrinsic_T","extrin_calib.Pcl"})
    node->declare_parameter(key,std::vector<double>{0,0,0});
  sensor_msgs::msg::PointCloud2 preview;
  auto sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("/fast_livo/map_preview",
    rclcpp::QoS(1).transient_local(),[&](const sensor_msgs::msg::PointCloud2 &msg){preview=msg;});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  {
    NavigationOutput output(node);
    for(int region=-1;region<=1;++region) {
      NavigationOutput::Cloud cloud;
      for(int n=0;n<200;++n) {pcl::PointXYZINormal p;p.x=region*100+n%10*.2;p.y=n/10*.2;p.z=1;p.intensity=n;cloud.push_back(p);}
      builtin_interfaces::msg::Time stamp;stamp.sec=region+5;
      output.publish(cloud,cloud,stamp,"base_link","odom");
    }
    const auto preview_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (preview.header.stamp.sec < 6 && std::chrono::steady_clock::now() < preview_deadline) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    check(preview.header.stamp.sec == 6 && preview.width > 0);
    pcl::PointCloud<pcl::PointXYZRGB> view;pcl::fromROSMsg(preview,view);
    check(view.size()<=40);bool left=false,right=false;
    for(const auto &p:view){left|=p.x < -90;right|=p.x>90;}check(left&&right);
    auto client=node->create_client<fast_livo::srv::SaveMap>("/fast_livo/save_map");
    check(client->wait_for_service(std::chrono::seconds(2)));
    auto request=std::make_shared<fast_livo::srv::SaveMap::Request>();
    request->save_path=std::string(root)+"/navigation.pcd";request->leaf_size=.05;
    // Save immediately while the archive worker still has freshly submitted
    // geometry. All prior input must be included without a caller-side sleep.
    NavigationOutput::Cloud tail;
    for(int n=0;n<200;++n) {pcl::PointXYZINormal p;p.x=200+n%10*.2;p.y=n/10*.2;p.z=1;p.intensity=n;tail.push_back(p);}
    builtin_interfaces::msg::Time tail_stamp;tail_stamp.sec=8;
    output.publish(tail,tail,tail_stamp,"base_link","odom");
    auto future=client->async_send_request(request);
    check(executor.spin_until_future_complete(future,std::chrono::seconds(20))==rclcpp::FutureReturnCode::SUCCESS);
    check(future.get()->success);
    pcl::PointCloud<pcl::PointXYZI> target;
    check(pcl::io::loadPCDFile(request->save_path,target)==0 && target.size()==800);
    left=right=false;for(const auto &p:target){left|=p.x < -90;right|=p.x>90;}check(left&&right);
    auto duplicate=client->async_send_request(request);
    check(executor.spin_until_future_complete(duplicate,std::chrono::seconds(5))==rclcpp::FutureReturnCode::SUCCESS);
    check(!duplicate.get()->success);
    // Invalid requests still fail; a coarser request remains supported.
    int index=0;
    for(double leaf:{0.,-1.,std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
      request->leaf_size=leaf;
      request->save_path=std::string(root)+"/invalid-"+std::to_string(index++)+".pcd";
      auto invalid=client->async_send_request(request);
      check(executor.spin_until_future_complete(invalid,std::chrono::seconds(5))==rclcpp::FutureReturnCode::SUCCESS);
      check(!invalid.get()->success && !std::filesystem::exists(request->save_path));
    }
    request->leaf_size=.4;request->save_path=std::string(root)+"/coarse.pcd";
    auto coarse=client->async_send_request(request);
    check(executor.spin_until_future_complete(coarse,std::chrono::seconds(20))==rclcpp::FutureReturnCode::SUCCESS);
    check(coarse.get()->success);
    check(pcl::io::loadPCDFile(request->save_path,target)==0 && !target.empty() && target.size()<800);
  }
  std::filesystem::remove_all(root);rclcpp::shutdown();
}
