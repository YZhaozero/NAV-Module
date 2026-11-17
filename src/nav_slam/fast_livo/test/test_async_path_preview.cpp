#include "async_path_preview.h"

#include <future>
#include <stdexcept>
#include <vector>

using Preview = fast_livo_preview::AsyncPathPreview;
using namespace std::chrono_literals;

static void require(bool condition) {
  if (!condition) throw std::runtime_error("Asynchronous path preview regression");
}
static Preview::Pose pose(int n, const char *frame = "map") {
  Preview::Pose p;
  p.header.frame_id = frame;
  p.header.stamp.sec = n;
  p.header.stamp.nanosec = 123;
  p.pose.position.x = n;
  return p;
}
struct Received {
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<Preview::Path> paths;
  std::vector<std::chrono::steady_clock::time_point> times;
  void add(Preview::Path::UniquePtr p) {
    std::lock_guard<std::mutex> lock(mutex);
    paths.push_back(std::move(*p));
    times.push_back(std::chrono::steady_clock::now());
    changed.notify_all();
  }
  Preview::Path waitLast(int last, const char *frame = "map") {
    std::unique_lock<std::mutex> lock(mutex);
    require(changed.wait_for(lock, 2s, [&] {
      return !paths.empty() && paths.back().header.stamp.sec == last && paths.back().header.frame_id == frame;
    }));
    return paths.back();
  }
};

int main() {
  Preview::Options options;
  options.period = 20ms;
  options.history_limit = 10;
  options.prune_count = 3;
  options.pending_limit = 100;
  Received received;
  std::atomic<bool> subscribed{false};
  Preview preview([&](Preview::Path::UniquePtr p) { received.add(std::move(p)); },
                  [&] { return subscribed.load(); }, options);
  // No subscriber: retain every pose, but do not invoke serialization/publish.
  for (int i = 1; i <= 13; ++i) require(preview.append(pose(i)));
  std::this_thread::sleep_for(60ms);
  require(preview.stats().published == 0);
  subscribed = true;
  auto path = received.waitLast(13);
  require(path.poses.size() == 10 && path.poses.front().pose.position.x == 4);
  require(path.header.stamp.nanosec == 123);
  for (std::size_t i = 0; i < path.poses.size(); ++i) require(path.poses[i].pose.position.x == i + 4);
  // New frame and reversed time each start an independent visible session.
  preview.append(pose(14, "odom"));
  path = received.waitLast(14, "odom");
  require(path.poses.size() == 1);
  preview.append(pose(2, "odom"));
  path = received.waitLast(2, "odom");
  require(path.poses.size() == 1);
  // The worker's cadence remains bounded even with frequent producer updates.
  for (int i = 3; i <= 18; ++i) { preview.append(pose(i, "odom")); std::this_thread::sleep_for(3ms); }
  received.waitLast(18, "odom");
  preview.stop();
  for (std::size_t i = 1; i < received.times.size(); ++i)
    require(received.times[i] - received.times[i - 1] >= options.period);
  require(!preview.append(pose(19)));

  // Deliberately block publication. Producer still completes, memory is bounded,
  // and overflow must publish the complete newest window without old segments.
  options.pending_limit = options.history_limit = 10;
  options.prune_count = 3;
  Received blocked_received;
  std::promise<void> entered, release;
  auto released = release.get_future().share();
  std::atomic<bool> first{true};
  Preview blocked([&](Preview::Path::UniquePtr p) {
    if (first.exchange(false)) { entered.set_value(); released.wait(); }
    blocked_received.add(std::move(p));
  }, [] { return true; }, options);
  blocked.append(pose(1));
  require(entered.get_future().wait_for(2s) == std::future_status::ready);
  auto producer = std::async(std::launch::async, [&] { for (int i = 2; i <= 42; ++i) blocked.append(pose(i)); });
  const auto ready = producer.wait_for(500ms);
  // Always release before asserting, so a regression cannot hang destruction.
  release.set_value();
  require(ready == std::future_status::ready);
  producer.get();
  path = blocked_received.waitLast(42);
  require(path.poses.size() == 10 && path.poses.front().pose.position.x == 33);
  for (std::size_t i = 0; i < path.poses.size(); ++i) require(path.poses[i].pose.position.x == i + 33);
  require(blocked.stats().peak_pending == 10 && blocked.stats().dropped_preview_poses == 31);
  blocked.stop();

  // Callback failures reach the owner instead of silently killing the thread.
  std::promise<void> failed;
  Preview broken([&](Preview::Path::UniquePtr) { failed.set_value(); throw std::runtime_error("publisher failed"); },
                 [] { return true; }, options);
  broken.append(pose(1));
  require(failed.get_future().wait_for(2s) == std::future_status::ready);
  broken.stop();
  bool threw = false;
  try { broken.rethrowIfFailed(); } catch (const std::runtime_error &) { threw = true; }
  require(threw);
}
