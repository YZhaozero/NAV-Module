#include "sensor_inbox.h"
#include <atomic>
#include <cassert>
#include <chrono>
#include <future>
#include <thread>
#include <vector>

int main() {
  SensorInbox inbox;
  std::vector<int> values;
  auto consumer = std::this_thread::get_id();
  // Receive during a deliberately stalled estimator; all 200 Hz samples survive
  // and estimator callbacks run only later on their owning thread.
  auto producer = std::async(std::launch::async, [&] {
    for (int i=0; i<100; ++i) {
      inbox.push(SensorInbox::Imu, 324, [&,i] {
        assert(std::this_thread::get_id() == consumer); values.push_back(i);
      });
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  assert(producer.wait_for(std::chrono::seconds(3)) == std::future_status::ready);
  producer.get();
  assert(values.empty() && inbox.stats().count[1] == 100);
  while (auto work = inbox.take()) work();
  assert(values.size() == 100);
  for (int i=0; i<100; ++i) assert(values[i] == i);

  SensorInbox bounded({{{2,20},{3,30},{2,20}}});
  values.clear();
  bounded.push(SensorInbox::Image,10,[&]{values.push_back(0);});
  bounded.push(SensorInbox::Imu,10,[&]{values.push_back(1);});
  bounded.push(SensorInbox::Lidar,10,[&]{values.push_back(2);});
  bounded.push(SensorInbox::Image,10,[&]{values.push_back(3);});
  bounded.push(SensorInbox::Image,10,[&]{values.push_back(4);});
  assert(bounded.stats().dropped[2] == 1);
  assert(bounded.stats().count[1] == 1); // Image pressure cannot evict IMU.
  while (auto work = bounded.take()) work();
  assert((values == std::vector<int>{1,2,3,4}));
  assert(bounded.stats().bytes[2] == 0);
  bounded.push(SensorInbox::Image,21,[]{});
  assert(bounded.stats().dropped[2] == 2 && !bounded.take());
  for(int i=0;i<4;++i) bounded.push(SensorInbox::Imu,10,[]{});
  bool error=false;
  try { bounded.take(); } catch(const std::runtime_error&) { error=true; }
  assert(error); // No integration through an unreported missing IMU interval.

  // The popped consumer does not retain the queue mutex.
  SensorInbox concurrent;
  concurrent.push(SensorInbox::Lidar,1,[&] {
    auto push=std::async(std::launch::async,[&]{concurrent.push(SensorInbox::Imu,1,[]{});});
    assert(push.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
    push.get();
  });
  concurrent.take()();
  assert(concurrent.stats().count[1] == 1);
  concurrent.fail(std::make_exception_ptr(std::runtime_error("receive failed")));
  error=false;
  try { concurrent.take(); } catch(const std::runtime_error&) { error=true; }
  assert(error);
}
