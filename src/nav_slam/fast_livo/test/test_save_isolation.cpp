#include "navigation_output.h"
#include <pcl/io/pcd_io.h>
#include <sys/syscall.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>

using Clock = std::chrono::steady_clock;
static std::mutex gate_mutex;
static std::condition_variable gate_wake;
static bool gate_enabled = false, gate_entered = false, gate_released = false;

// Pause a REAL completed PCD fsync. This test-only syscall interposition
// reproduces a slow exporter without adding hooks or sleeps to production.
extern "C" int fdatasync(int fd) {
  char link[64], target[2048];
  snprintf(link,sizeof(link),"/proc/self/fd/%d",fd);
  const auto n=readlink(link,target,sizeof(target)-1);
  if(n>0) {
    target[n]=0;
    if(std::strstr(target,"/nav-save-isolation-") && std::strstr(target,".pcd.") &&
       !std::strstr(target,".commit.json")) {
      std::unique_lock<std::mutex> lock(gate_mutex);
      if(gate_enabled) {
        gate_entered=true;gate_wake.notify_all();
        gate_wake.wait(lock,[]{return gate_released;});
      }
    }
  }
  return syscall(SYS_fdatasync,fd);
}
static void releaseGate() {
  std::lock_guard<std::mutex> lock(gate_mutex);gate_released=true;gate_wake.notify_all();
}
static void check(bool ok,const char *why) {if(!ok)throw std::runtime_error(why);}
template<class Predicate> static bool until(Predicate predicate,double seconds) {
  const auto end=Clock::now()+std::chrono::duration<double>(seconds);
  do {if(predicate())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  while(Clock::now()<end);
  return predicate();
}
static NavigationOutput::Cloud::ConstPtr cloud(int frame) {
  auto result=std::make_shared<NavigationOutput::Cloud>();
  for(int i=0;i<100;++i) {
    pcl::PointXYZINormal p{};p.x=frame*20.f+i*.125f;p.y=.25f;p.z=1.f;p.intensity=frame;
    result->push_back(p);
  }
  return result;
}
static builtin_interfaces::msg::Time stamp(int seconds) {
  builtin_interfaces::msg::Time value;value.sec=seconds;return value;
}
struct Cleanup {
  std::atomic<bool> &stop;
  rclcpp::executors::SingleThreadedExecutor &executor;
  std::thread &thread;
  ~Cleanup() {releaseGate();stop=true;executor.cancel();if(thread.joinable())thread.join();}
};
struct NavigationOutputTestAccess {
  static void drain(NavigationOutput &output){output.drain();}
};
static size_t chunks(const std::string &root) {
  size_t count=0;
  for(const auto &entry:std::filesystem::recursive_directory_iterator(root))
    if(entry.path().extension()==".bin")++count;
  return count;
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  char root[]="/tmp/nav-save-isolation-XXXXXX";check(mkdtemp(root),"temp directory failed");
  auto node=std::make_shared<rclcpp::Node>("save_isolation",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("navigation.enabled",true),
        rclcpp::Parameter("navigation.map_voxel_size",.05),
        rclcpp::Parameter("navigation.max_map_voxels",200),
        rclcpp::Parameter("navigation.map_pending_bytes",int64_t(32*1024)),
        rclcpp::Parameter("navigation.map_checkpoint_period",3600.),
        rclcpp::Parameter("navigation.map_spool_directory",std::string(root))}));
  node->declare_parameter("navigation.body_frame","base_link");
  for(auto key:{"extrin_calib.extrinsic_R","extrin_calib.Rcl"})
    node->declare_parameter(key,std::vector<double>{1,0,0,0,1,0,0,0,1});
  for(auto key:{"extrin_calib.extrinsic_T","extrin_calib.Pcl"})
    node->declare_parameter(key,std::vector<double>{0,0,0});
  rclcpp::executors::SingleThreadedExecutor main_executor;
  main_executor.add_node(node);
  {
    auto output=std::make_unique<NavigationOutput>(node);
    auto first=cloud(0);output->publish(first,first,stamp(1),"base_link","odom");
    NavigationOutputTestAccess::drain(*output);
    auto client=node->create_client<fast_livo::srv::SaveMap>("/fast_livo/save_map");
    check(client->wait_for_service(std::chrono::seconds(3)),"service unavailable");
    std::atomic<bool> produce{false},stop{false};
    std::atomic<int> frames{0};
    auto timer=node->create_wall_timer(std::chrono::milliseconds(10),[&] {
      if(!produce || frames>=100)return;
      const int frame=++frames;auto points=cloud(frame);
      output->publish(points,points,stamp(frame+1),"base_link","odom");
    });
    std::exception_ptr spin_failure;
    std::thread spinning([&] {
      try {while(!stop && rclcpp::ok())main_executor.spin_once(std::chrono::milliseconds(20));}
      catch(...){spin_failure=std::current_exception();}
    });
    Cleanup cleanup{stop,main_executor,spinning};
    {std::lock_guard<std::mutex> lock(gate_mutex);gate_enabled=true;gate_released=false;gate_entered=false;}
    auto request=std::make_shared<fast_livo::srv::SaveMap::Request>();
    request->save_path=std::string(root)+"/prefix.pcd";request->leaf_size=.05;
    auto saved=client->async_send_request(request);
    {
      std::unique_lock<std::mutex> lock(gate_mutex);
      check(gate_wake.wait_for(lock,std::chrono::seconds(5),[]{return gate_entered;}),"export did not start");
    }
    const auto before=chunks(root);
    produce=true;
    const bool default_executor_progress=until([&]{return frames>=60;},3.);
    const bool archive_progress=chunks(root)>before+10;
    // Release before assertions, so a failing regression cannot leave a join
    // hung on its deliberate fault injection.
    produce=false;releaseGate();
    check(default_executor_progress,"export blocked the estimator/default callback group");
    check(archive_progress,"export blocked the checkpoint worker/new input archive");
    check(until([&]{return saved.wait_for(std::chrono::milliseconds(0))==std::future_status::ready;},5.),"save response timed out");
    check(saved.get()->success,"prefix save failed");
    NavigationOutputTestAccess::drain(*output);
    pcl::PointCloud<pcl::PointXYZI> points;
    check(pcl::io::loadPCDFile(request->save_path,points)==0 && points.size()==100,
          "save snapshot incorporated frames submitted after its checkpoint");
    check(std::filesystem::exists(request->save_path+".commit.json"),"durable receipt missing");
    for(const auto &entry:std::filesystem::directory_iterator(root))
      if(entry.is_directory())check(!std::filesystem::exists(entry.path()/"exported.txt"),
          "prefix export hid newer capture checkpoints");
    auto complete=std::make_shared<fast_livo::srv::SaveMap::Request>();
    complete->save_path=std::string(root)+"/complete.pcd";complete->leaf_size=.05;
    auto all=client->async_send_request(complete);
    check(until([&]{return all.wait_for(std::chrono::milliseconds(0))==std::future_status::ready;},10.),"full save response timed out");
    check(all.get()->success,"full save failed");
    check(pcl::io::loadPCDFile(complete->save_path,points)==0 && points.size()==size_t(frames+1)*100,
          "continued capture lost geometry while the first save was stalled");
    // Shut ROS down while an export is genuinely in flight. Destruction may
    // wait for that finite operation, but must keep the archive worker alive
    // until the snapshot receipt is finished and then join without deadlock.
    {std::lock_guard<std::mutex> lock(gate_mutex);gate_entered=false;gate_released=false;}
    auto shutdown_request=std::make_shared<fast_livo::srv::SaveMap::Request>();
    shutdown_request->save_path=std::string(root)+"/shutdown.pcd";shutdown_request->leaf_size=.05;
    auto shutdown_result=client->async_send_request(shutdown_request);
    {
      std::unique_lock<std::mutex> lock(gate_mutex);
      check(gate_wake.wait_for(lock,std::chrono::seconds(5),[]{return gate_entered;}),"shutdown export did not start");
    }
    std::string old_directory;
    for(const auto &entry:std::filesystem::directory_iterator(root))
      if(entry.is_directory())old_directory=entry.path().string();
    auto next=cloud(200);
    output->publish(next,next,stamp(1),"base_link","next_odom");
    NavigationOutputTestAccess::drain(*output);
    bool old_locked=false;
    try {MapSpool::recover(old_directory);}catch(const std::exception &){old_locked=true;}
    check(old_locked,"generation reset released an in-flight snapshot's recovery lock");
    stop=true;main_executor.cancel();spinning.join();
    if(spin_failure)std::rethrow_exception(spin_failure);
    timer.reset();
    rclcpp::shutdown();
    std::atomic<bool> destroyed{false};
    std::thread destroying([&]{output.reset();destroyed=true;});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const bool waited_for_save=!destroyed;
    const auto start=Clock::now();releaseGate();destroying.join();
    check(waited_for_save,"destructor detached a still-running save");
    check(std::chrono::duration<double>(Clock::now()-start).count()<2.,"save executor shutdown hung");
    check(!std::filesystem::exists(shutdown_request->save_path),"cancelled export was committed after shutdown");
    check(chunks(root)>0,"cancelled export lost recoverable checkpoints");
    bool next_recovered=false;
    for(const auto &entry:std::filesystem::directory_iterator(root)) {
      if(!entry.is_directory() || entry.path()==old_directory)continue;
      auto recovery=MapSpool::recover(entry.path().string());
      const auto recovered_path=std::string(root)+"/next-generation.pcd";
      recovery->exportPcd(recovered_path,.05,[](const auto&){});
      pcl::PointCloud<pcl::PointXYZI> recovered;
      check(pcl::io::loadPCDFile(recovered_path,recovered)==0 && recovered.size()==100,
            "save cancellation mixed or lost the new coordinate generation");
      for(const auto &point:recovered)check(point.x>=4000.f,"old generation contaminated recovery");
      next_recovered=true;
    }
    check(next_recovered,"shutdown did not checkpoint the new generation");
    std::cout<<"save isolation passed: "<<frames<<" frames archived during blocked export; "
             <<points.size()<<" final measured points; default executor remained live\n";
  }
  std::filesystem::remove_all(root);
}
