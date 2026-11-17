#include "lidar_slice_end_cache.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

struct Point { int id; float curvature; };
struct Cloud {
  using Ptr = std::shared_ptr<Cloud>;
  using ConstPtr = std::shared_ptr<const Cloud>;
  std::vector<Point> points;
  void push_back(const Point &point) { points.push_back(point); }
};
using Cache = fast_livo_schedule::LidarSliceEndCache<Cloud::ConstPtr>;
static double oldCarriedEnd(const Cloud &cloud, double anchor) {
  double end = anchor;
  for (const auto &point : cloud.points)
    end = std::max(end, anchor + point.curvature / 1000.0);
  return end;
}
static std::size_t checks = 0;
static void require(bool value, int line) {
  ++checks;
  if (!value) throw std::runtime_error("slice end cache regression at line " + std::to_string(line));
}
#define CHECK(value) require((value), __LINE__)

int main() {
  for (double epoch : {10., 1793197026.}) {
    Cache raw, carried;
    auto scan = std::make_shared<Cloud>();
    scan->points = {{1, 75.f}, {2, 100.4f}, {3, 0.f}, {4, 50.f},
        {5, std::numeric_limits<float>::quiet_NaN()},
        {6, std::numeric_limits<float>::infinity()}};
    const double end = fast_livo_schedule::lidarSliceEnd(*scan, epoch);
    CHECK(raw.get(scan, epoch) == end);
    for (int wait = 0; wait < 5000; ++wait) CHECK(raw.get(scan, epoch) == end);
    CHECK(raw.scans() == 1 && raw.hits() == 5000);

    // A new scan, even at an equal timestamp and identical size, is a miss.
    auto newer = std::make_shared<Cloud>(*scan);
    newer->points[1].curvature = 105.f;
    CHECK(raw.get(newer, epoch) == fast_livo_schedule::lidarSliceEnd(*newer, epoch));
    CHECK(raw.scans() == 2);
    // Timestamp rebasing/rollback is independent of object identity.
    CHECK(raw.get(newer, epoch - 10.) == fast_livo_schedule::lidarSliceEnd(*newer, epoch - 10.));
    CHECK(raw.scans() == 3);

    auto future = std::make_shared<Cloud>();
    Cloud current;
    fast_livo_schedule::appendLidarSlice(*scan, epoch, epoch, epoch + .05, current, *future);
    CHECK(current.points.size() + future->points.size() == 4); // invalid points excluded by production slicing.
    CHECK(carried.get(future, epoch + .05) == fast_livo_schedule::lidarSliceEnd(*future, epoch + .05));
    CHECK(carried.get(future, epoch + .05) == oldCarriedEnd(*future, epoch + .05));
    const auto old_count = future->points.size();
    // The carried object is reused across cuts; invalidate BEFORE mutating.
    carried.invalidate();
    Cloud previous; previous.points.swap(future->points);
    Cloud second;
    fast_livo_schedule::appendLidarSlice(previous, epoch + .05, epoch + .05,
        epoch + .075, second, *future);
    CHECK(second.points.size() + future->points.size() == old_count);
    CHECK(carried.get(future, epoch + .075) == fast_livo_schedule::lidarSliceEnd(*future, epoch + .075));
    CHECK(carried.scans() == 2);
    // Same object / same size / same allocation / same stamp can change after
    // recovery; the explicit invalidation contract must cover this case.
    carried.invalidate();
    for (auto &point : future->points) point.curvature += 90.f;
    CHECK(carried.get(future, epoch + .075) == fast_livo_schedule::lidarSliceEnd(*future, epoch + .075));
    CHECK(carried.scans() == 3);
    // Reset releases ownership and handles an empty, then repopulated cloud.
    std::weak_ptr<const Cloud> retained = newer;
    newer.reset();
    CHECK(!retained.expired());
    raw.invalidate();
    CHECK(retained.expired());
    carried.invalidate(); future->points.clear();
    CHECK(carried.get(future, epoch) == epoch);
    future->push_back({9, 40.f});
    CHECK(carried.get(future, epoch) == epoch + .04);
    CHECK(carried.get({}, epoch) == epoch);
  }

  // Random out-of-order curvature and cut sequences use the exact existing
  // max/split arithmetic, including large epoch roundoff and invalid input.
  std::mt19937 random(20261029);
  std::uniform_real_distribution<float> offset(-25.f, 175.f);
  for (int trial = 0; trial < 1000; ++trial) {
    const double epoch = trial % 2 ? 1793197026. : 10.;
    Cache cache;
    auto cloud = std::make_shared<Cloud>();
    for (int index = 0; index < 333; ++index) cloud->push_back({index, offset(random)});
    CHECK(cache.get(cloud, epoch) == fast_livo_schedule::lidarSliceEnd(*cloud, epoch));
    std::shuffle(cloud->points.begin(), cloud->points.end(), random);
    cache.invalidate();
    CHECK(cache.get(cloud, epoch) == fast_livo_schedule::lidarSliceEnd(*cloud, epoch));
    for (int cut = 0; cut < 4; ++cut) {
      const double state = epoch + .05 * cut, next = state + .05;
      cache.invalidate();
      Cloud old; old.points.swap(cloud->points);
      Cloud used;
      fast_livo_schedule::appendLidarSlice(old, state, state, next, used, *cloud);
      CHECK(cache.get(cloud, next) == fast_livo_schedule::lidarSliceEnd(*cloud, next));
      CHECK(cache.get(cloud, next) == oldCarriedEnd(*cloud, next));
      CHECK(cache.get(cloud, next) == fast_livo_schedule::lidarSliceEnd(*cloud, next));
    }
  }

  auto cloud = std::make_shared<Cloud>();
  for (int index = 0; index < 15000; ++index) cloud->push_back({index, offset(random)});
  Cache cache; const double epoch = 1793197026.;
  volatile double sink = 0.; constexpr int repetitions = 5000;
  const auto a = std::chrono::steady_clock::now();
  for (int i = 0; i < repetitions; ++i) sink += fast_livo_schedule::lidarSliceEnd(*cloud, epoch);
  const auto b = std::chrono::steady_clock::now();
  for (int i = 0; i < repetitions; ++i) sink += cache.get(cloud, epoch);
  const auto c = std::chrono::steady_clock::now();
  std::cout << "checks=" << checks << " full_scans=" << cache.scans()
            << " cache_hits=" << cache.hits()
            << " baseline_ms=" << std::chrono::duration<double, std::milli>(b-a).count()
            << " cache_ms=" << std::chrono::duration<double, std::milli>(c-b).count()
            << " sink=" << sink << '\n';
}
