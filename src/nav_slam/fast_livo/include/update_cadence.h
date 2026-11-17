#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace fast_livo_schedule {
constexpr double kMaxLioInterval = 0.1;
constexpr double kMinImageLioInterval = 0.005;
// Epoch seconds in double have ~0.24 us spacing in 2026. Compare cut
// boundaries with rounding tolerance, without changing measurement timestamps.
constexpr double kTimeTolerance = 1e-6;
inline void validate(double lio, double vio) {
  if (!std::isfinite(lio) || !std::isfinite(vio) || lio < .005 ||
      lio > kMaxLioInterval || vio < 0. || vio > 1.)
    throw std::invalid_argument("Invalid LIO/VIO update interval");
}
inline bool skipImage(double image, double last_lio, double last_vio,
                      double lio_interval, double vio_interval) {
  // The nominal live cadence is a load budget, not a phase constraint.
  // Requiring 95 ms after a 100 ms fallback leaves only a 5 ms image window
  // and can starve an asynchronous 15 Hz camera forever. Permit a short LIO
  // cut to align with an eligible visual measurement, retaining the existing
  // 5 ms guard against degenerate slices and the independent VIO rate limit.
  return image + kTimeTolerance < last_lio + std::min(lio_interval, kMinImageLioInterval) ||
         (last_vio >= 0. && image + kTimeTolerance < last_vio + vio_interval);
}
inline double imageDecodeInterval(double lio_interval, double vio_interval) {
  // Throttle selected visual measurements before decoding, instead of first
  // reducing 15 Hz to 7.5 Hz and then losing every other frame to the VIO limit.
  return std::max(.02, std::max(lio_interval, vio_interval));
}
inline double nextLioTime(double last_lio, double latest_image, double last_vio,
                         double lio_interval) {
  // An image arriving within 5 ms of an already-consumed fallback state cannot
  // be fused there. Shift the next cut by 5 ms once, so subsequent images at
  // that camera phase do not remain coincident forever. Do not rephase after
  // an image that was actually fused. No timestamp or IMU sample is changed.
  const bool near_unfused_image = lio_interval > kMinImageLioInterval &&
      latest_image > last_vio + kTimeTolerance &&
      std::abs(latest_image - last_lio) <= kMinImageLioInterval + kTimeTolerance;
  return last_lio + kMaxLioInterval - (near_unfused_image ? kMinImageLioInterval : 0.);
}
inline bool waitForImage(double last_lio, double next_lio, double latest_image,
                         double newest_imu, double lio_interval, double vio_interval) {
  if (vio_interval <= 0. || latest_image < 0.) return false;
  const double expected = latest_image + imageDecodeInterval(lio_interval, vio_interval);
  // A selected frame may be delivered shortly after its measurement epoch.
  // Do not consume that exact epoch just before it arrives. The sensor clock
  // bounds this wait to 50 ms; camera loss then resumes the same <=100 ms LIO
  // cuts without discarding IMU or extrapolating. Offline scheduling is unchanged.
  constexpr double max_arrival_wait = .05;
  return expected > last_lio + kMinImageLioInterval - kTimeTolerance &&
         expected <= next_lio + kTimeTolerance &&
         newest_imu + kTimeTolerance < expected + max_arrival_wait;
}
// Only the visual observation is skipped. LIO still advances at <=100 ms
// using every buffered LiDAR return and IMU measurement with its original time.
constexpr double kImuGapWarnS = .2;
constexpr double kImuGapWindowS = 10.;
constexpr double kImuFatalGapS = 2.;
constexpr std::size_t kImuGapLimit = 4;
// A short hole in the IMU stream cannot be integrated, but it is
// recoverable: the caller drops the stale queues and restarts the LIO
// interval at the newest sample. Only a long hole, or several holes in one
// window, means the stream itself is broken and localization must restart.
inline bool imuGapIsFatal(double gap, std::size_t gaps_in_window) {
  if (!std::isfinite(gap) || gap < 0.) return true;
  return gap > kImuFatalGapS || gaps_in_window > kImuGapLimit;
}
}
