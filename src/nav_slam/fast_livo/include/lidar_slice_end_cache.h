#pragma once

#include "lidar_slice.h"
#include <cstddef>
#include <cstdint>

namespace fast_livo_schedule {
// A one-cloud memo for wait loops. Queued raw scans are immutable; carried
// slices must call invalidate() BEFORE swap/clear/append changes their points.
// Holding a const shared pointer prevents allocator address reuse from making
// a different cloud look like a cache hit. This never reorders or drops points.
template <class ConstCloudPointer>
class LidarSliceEndCache {
 public:
  double get(const ConstCloudPointer &cloud, double scan_time) {
    if (!cloud) return scan_time;
    if (cloud_ == cloud && scan_time_ == scan_time &&
        point_count_ == cloud->points.size() && data_ == cloud->points.data()) {
      ++hits_;
      return end_time_;
    }
    const double end = lidarSliceEnd(*cloud, scan_time);
    cloud_ = cloud;
    scan_time_ = scan_time;
    point_count_ = cloud->points.size();
    data_ = cloud->points.data();
    end_time_ = end;
    ++scans_;
    return end;
  }

  void invalidate() {
    cloud_.reset();
    data_ = nullptr;
    point_count_ = 0;
  }

  std::uint64_t scans() const { return scans_; }
  std::uint64_t hits() const { return hits_; }

 private:
  ConstCloudPointer cloud_;
  const void *data_ = nullptr;
  std::size_t point_count_ = 0;
  double scan_time_ = 0., end_time_ = 0.;
  std::uint64_t scans_ = 0, hits_ = 0;
};
}  // namespace fast_livo_schedule
