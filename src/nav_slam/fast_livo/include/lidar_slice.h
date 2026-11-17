#pragma once

#include <algorithm>
#include <cmath>

namespace fast_livo_schedule {
// Point curvature is acquisition time in milliseconds relative to its scan
// header. A delayed scan may straddle a state already advanced after an IMU
// gap; keep its covered tail, never reinterpret its old prefix at the new pose.
constexpr double kLidarSliceToleranceMs = .001;

template <class Cloud>
bool lidarSliceExpired(const Cloud &cloud, double scan_time, double state_time) {
  // Do not assume packet/point order. Standard and Livox sources may differ.
  const double offset_ms = (scan_time - state_time) * 1000.0;
  for (const auto &point : cloud.points) {
    const double relative_ms = offset_ms + point.curvature;
    if (std::isfinite(relative_ms) && relative_ms >= -kLidarSliceToleranceMs) return false;
  }
  return true;
}

template <class Cloud>
double lidarSliceEnd(const Cloud &cloud, double scan_time) {
  double end_ms = 0.;
  for (const auto &point : cloud.points)
    if (std::isfinite(point.curvature)) end_ms = std::max(end_ms, double(point.curvature));
  return scan_time + end_ms / 1000.0;
}

template <class Cloud>
void appendLidarSlice(const Cloud &source, double scan_time, double state_time,
                      double cut_time, Cloud &current, Cloud &future,
                      bool include_cut = false) {
  const double state_offset_ms = (scan_time - state_time) * 1000.0;
  const double cut_offset_ms = (scan_time - cut_time) * 1000.0;
  const float scan_cut_ms = (cut_time - scan_time) * 1000.0;
  for (auto point : source.points) {
    const double relative_ms = state_offset_ms + point.curvature;
    if (!std::isfinite(relative_ms) || relative_ms < -kLidarSliceToleranceMs) continue;
    if (point.curvature < scan_cut_ms ||
        (include_cut && cut_offset_ms + point.curvature <= kLidarSliceToleranceMs)) {
      point.curvature = std::max(0.0, relative_ms);
      current.push_back(point);
    } else {
      point.curvature = std::max(0.0, cut_offset_ms + point.curvature);
      future.push_back(point);
    }
  }
}
}  // namespace fast_livo_schedule
