#include "lidar_slice.h"
#include <iostream>
#include <stdexcept>
#include <vector>

struct Point { int id; float curvature; };
struct Cloud {
  std::vector<Point> points;
  bool empty() const { return points.empty(); }
  void push_back(const Point &point) { points.push_back(point); }
};
static void require(bool value) {
  if (!value) throw std::runtime_error("LiDAR slice time regression");
}

int main() {
  for (double epoch : {100., 1793077866.}) {
    Cloud scan{{{1, 0.f}, {2, 25.f}, {3, 50.f}, {4, 75.f}, {5, 100.f}, {6, 125.f}}};
    Cloud current, future;
    // Ordinary consecutive cuts retain every point exactly once.
    fast_livo_schedule::appendLidarSlice(scan, epoch, epoch, epoch + .075, current, future);
    require(current.points.size() + future.points.size() == 6);
    for (const auto &point : current.points)
      require(std::abs(point.curvature - (point.id - 1) * 25.f) < .001f);
    Cloud next, retained;
    fast_livo_schedule::appendLidarSlice(future, epoch + .075, epoch + .075,
                                       epoch + .150, next, retained);
    require(retained.empty() && current.points.size() + next.points.size() == 6);
    for (const auto &point : next.points)
      require(std::abs(point.curvature - ((point.id - 1) * 25.f - 75.f)) < .001f);

    // The state jumps to the first covered IMU after a gap. An older scan
    // arrives late: its pre-gap prefix is unusable, its tail remains measured.
    current.points.clear(); future.points.clear();
    fast_livo_schedule::appendLidarSlice(scan, epoch, epoch + .050,
                                       epoch + .100, current, future);
    require(current.points.size() + future.points.size() == 4);
    for (const auto &point : current.points) {
      require(point.id >= 3 && point.curvature >= 0.f);
      require(std::abs(point.curvature - ((point.id - 1) * 25.f - 50.f)) < .001f);
    }
    for (const auto &point : future.points) require(point.id >= 5 && point.curvature >= 0.f);
    require(fast_livo_schedule::lidarSliceExpired(scan, epoch - 1., epoch + .05));
    require(!fast_livo_schedule::lidarSliceExpired(scan, epoch, epoch + .05));
    Cloud unordered{{{1, 125.f}, {2, 0.f}}};
    require(!fast_livo_schedule::lidarSliceExpired(unordered, epoch, epoch + .05));
    require(std::abs(fast_livo_schedule::lidarSliceEnd(unordered, epoch) - (epoch + .125)) < 1e-6);
    Cloud expired_current, expired_future;
    fast_livo_schedule::appendLidarSlice(scan, epoch - 1., epoch + .05,
                                       epoch + .15, expired_current, expired_future);
    require(expired_current.empty() && expired_future.empty());
    Cloud whole, no_tail;
    fast_livo_schedule::appendLidarSlice(scan, epoch, epoch + .05,
        epoch + .125, whole, no_tail, true);
    require(whole.points.size() == 4 && no_tail.empty());
    require(whole.points.front().id == 3 && whole.points.back().id == 6);
  }
  std::cout << "LiDAR slices preserve covered tails and reject expired points\n";
}
