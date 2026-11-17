#include "async_rgb_preview.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

using namespace fast_livo_rgb;
using namespace std::chrono_literals;
static void require(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
static void await(const std::function<bool()> &predicate, const char *message) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(1ms);
  require(predicate(), message);
}
static GeometryCloud::ConstPtr cloud(float x) {
  auto cloud = std::make_shared<GeometryCloud>();
  pcl::PointXYZINormal point{}; point.x = x; point.y = 0; point.z = 5;
  cloud->push_back(point); return cloud;
}
static std::shared_ptr<PreviewInput> input(int time, std::uint64_t epoch = 0) {
  auto value = std::make_shared<PreviewInput>();
  value->camera.width = 64; value->camera.height = 48;
  value->camera.fx = value->camera.fy = 40; value->camera.cx = 32; value->camera.cy = 24;
  value->rgb = cv::Mat(48, 64, CV_8UC3, cv::Scalar(10, 20, 30));
  value->annotated = cv::Mat(48, 64, CV_8UC3, cv::Scalar(40, 50, 60));
  value->geometry = {cloud(-.1), cloud(0), cloud(.1)};
  value->stamp.sec = time; value->stamp.nanosec = 123456789;
  value->generation = epoch; value->world_frame = "odom";
  return value;
}

int main() {
  for (int bits = 0; bits < 8; ++bits)
    require(asynchronousPreviewAllowed(bits & 1, bits & 2, bits & 4, true) == (bits == 0),
            "Research export inherited lossy display mode");
  require(!asynchronousPreviewAllowed(false, false, false, false), "Unsupported camera entered async mode");
  GeometryWindow window;
  for (int n = 0; n < 3; ++n) { window.begin(1. + .1*n, 0, "odom"); window.append(cloud((n-1)*.1)); }
  require(window.blocks() == 3, "Intermediate LIO slices were omitted");
  auto snapshot = input(7); snapshot->geometry = window.take();
  require(window.empty() && window.bytes() == 0, "Consumed geometry was retained");
  window.begin(2., 0, "odom"); window.append(cloud(0)); window.begin(2.6, 0, "odom");
  require(window.empty(), "Old 0.5-second geometry window survived");
  window.append(cloud(0)); window.begin(2.6, 1, "other");
  require(window.empty(), "Old coordinate generation survived");
  GeometryWindow bounded(2 * sizeof(GeometryCloud::PointType), 2);
  bounded.begin(0., 0, "odom"); for (int i=0; i<100; ++i) bounded.append(cloud(float(i)));
  require(bounded.blocks() == 2 && bounded.discardedPreviewBlocks() == 98, "Geometry display window unbounded");

  std::atomic<std::uint64_t> generation{0};
  std::atomic<int> results{0}, failed{0};
  std::mutex result_mutex;
  PreviewResult received;
  {
    AsyncRgbPreview worker([&](const PreviewResult &value) {
      std::lock_guard<std::mutex> lock(result_mutex); received = value; ++results;
    }, [&] { return generation.load(); }, [&](std::exception_ptr) { ++failed; });
    // The producer moves to freshly allocated next-frame Mats and a different
    // pose/calibration; the submitted snapshot must retain its original frame.
    cv::Mat previous_pixels = snapshot->rgb;
    worker.submit(snapshot);
    previous_pixels = cv::Mat(48, 64, CV_8UC3, cv::Scalar(250, 250, 250));
    snapshot.reset();
    await([&] { return results.load() == 1; }, "RGB worker did not publish");
    {
      std::lock_guard<std::mutex> lock(result_mutex);
      require(received.cloud->size() == 3, "RGB did not retain intermediate LIO geometry");
      require(received.stamp.sec == 7 && received.stamp.nanosec == 123456789,
              "Image was relabeled with publication wall time");
      for (const auto &p : *received.cloud) require(p.r == 30 && p.g == 20 && p.b == 10, "Frame pixels changed");
      require(received.annotated.at<cv::Vec3b>(0,0) == cv::Vec3b(40,50,60), "Annotated image changed");
    }
    auto bad = input(8); bad->rgb = cv::Mat(48,64,CV_8UC1); worker.submit(bad);
    await([&] { return failed.load() == 1; }, "Bad frame did not report recoverable error");
    generation = 1;
    worker.submit(input(9, 0)); std::this_thread::sleep_for(30ms);
    require(results.load() == 1, "Stale generation published into new map");
    worker.submit(input(10, 1));
    await([&] { return results.load() == 2; }, "Worker failed to recover after invalid image");
    worker.stop();
  }

  // Hold a real colorized result at its publisher. Estimator submission still
  // finishes and only one most recent unrendered snapshot is retained.
  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  bool entered = false, released = false;
  std::vector<int> stamps;
  {
    AsyncRgbPreview worker([&](const PreviewResult &value) {
      std::unique_lock<std::mutex> lock(gate_mutex);
      stamps.push_back(value.stamp.sec); entered = true; gate_cv.notify_all();
      gate_cv.wait(lock, [&] { return released; });
    }, [] { return 0; });
    struct Release { std::mutex &m; std::condition_variable &cv; bool &value;
      ~Release() { std::lock_guard<std::mutex> lock(m); value = true; cv.notify_all(); }
    } release{gate_mutex, gate_cv, released};
    worker.submit(input(1));
    { std::unique_lock<std::mutex> lock(gate_mutex); require(gate_cv.wait_for(lock, 3s, [&] { return entered; }), "Blocked sink missing"); }
    const auto start = std::chrono::steady_clock::now();
    for (int i = 2; i <= 101; ++i) worker.submit(input(i));
    require(std::chrono::steady_clock::now() - start < 500ms, "RGB publication blocked estimator submit");
    require(worker.stats().replaced == 99 && worker.stats().active && worker.stats().pending,
            "Pending RGB frame queue was not bounded");
    { std::lock_guard<std::mutex> lock(gate_mutex); released = true; gate_cv.notify_all(); }
    await([&] { return worker.stats().completed == 2; }, "Newest RGB frame not processed");
    worker.stop();
    require(stamps == std::vector<int>({1,101}), "RGB worker replayed stale display backlog");
  }
  std::cout << "Asynchronous RGB preview: geometry, generation, recovery, legacy gates and bounded sink passed\n";
}
