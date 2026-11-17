#include "async_pose_output.h"
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <future>
static void require(bool ok) { if (!ok) throw std::runtime_error("pose output bounds/lifetime failed"); }
int main() {
  auto entered=std::make_shared<std::promise<void>>();
  auto release=std::make_shared<std::promise<void>>();
  auto unblock=release->get_future().share();
  auto completed=std::make_shared<std::atomic<int>>(0);
  auto output=std::make_unique<AsyncPoseOutput>([entered,unblock,completed](const auto &) {
    if (!completed->load()) { entered->set_value(); unblock.wait(); }
    ++*completed;
  },2);
  AsyncPoseOutput::Pose pose; pose.header.stamp.sec=10;
  require(output->submit(pose));
  require(entered->get_future().wait_for(std::chrono::seconds(2))==std::future_status::ready);
  require(output->submit(pose)); require(output->submit(pose));
  require(!output->submit(pose));
  auto s=output->stats(); require(s.pending==2 && s.rejected==1 && s.failed);
  auto begin=std::chrono::steady_clock::now();
  require(!output->stop(std::chrono::milliseconds(30)));
  output.reset();
  require(std::chrono::steady_clock::now()-begin<std::chrono::milliseconds(300));
  release->set_value();
  for (int i=0;i<200 && completed->load()<3;++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  require(completed->load()==3); // Captured state safely outlives the mapper.
  AsyncPoseOutput failing([](const auto &) { throw std::runtime_error("DDS failed"); });
  require(failing.submit(pose));
  for (int i=0;i<200 && !failing.stats().failed;++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
  require(failing.stats().failed && failing.stats().completed==0);
  require(!failing.submit(pose)); require(failing.stop());
  std::cout<<"bounded queue, explicit loss, bounded shutdown and worker lifetime passed\n";
}
