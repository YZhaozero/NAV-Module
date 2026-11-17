#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fast_livo_schedule {

// One estimator-owned controller. Its clock is monotonic wall time, while
// ready_backlog is a difference of sensor measurement times. Neither changes
// message stamps, IMU integration, LiDAR slicing or the archival stream.
class AdaptiveLoad {
 public:
  void enable(bool enabled) {
    enabled_ = enabled;
    if (!enabled) { level_ = 0; overload_s_ = healthy_s_ = 0.; }
  }
  bool enabled() const { return enabled_; }
  unsigned level() const { return level_; }
  std::uint64_t changes() const { return changes_; }
  const char *reason() const { return reason_; }
  double computeRatio() const { return compute_ratio_; }
  double readyBacklog() const { return ready_backlog_; }

  void observe(double elapsed, double compute_s, double ready_backlog,
               std::size_t lio_cycles) {
    if (!enabled_) { reason_ = "disabled"; return; }
    if (!std::isfinite(elapsed) || !std::isfinite(compute_s) ||
        !std::isfinite(ready_backlog) || elapsed < .5 || compute_s < 0.) {
      overload_s_ = healthy_s_ = 0.; reason_ = "invalid_window"; return;
    }
    compute_ratio_ = compute_s / elapsed;
    ready_backlog_ = std::max(0., ready_backlog);
    // Waiting for delayed input, disk backpressure and a stale pose alone do
    // not constitute compute pressure. Require available, covered work too.
    // Sustained estimator work close to a full core leaves no headroom for
    // bursts. Act before a queue has to accumulate; input/output waiting is
    // excluded by the caller's compute-only accounting.
    const bool proactive = lio_cycles >= 3 && compute_ratio_ >= .85;
    const bool overloaded = proactive || (lio_cycles >= 1 && compute_ratio_ >= .78 && ready_backlog_ >= .20);
    const bool severe = overloaded && elapsed >= 1. && compute_ratio_ >= .90 && ready_backlog_ >= .80;
    const bool healthy = lio_cycles >= 3 && compute_ratio_ <= .60 && ready_backlog_ <= .12;
    // A single long callback/window must never jump several levels.
    const double evidence_s = std::min(elapsed, 1.1);
    overload_s_ = overloaded ? overload_s_ + evidence_s : 0.;
    healthy_s_ = healthy ? healthy_s_ + evidence_s : 0.;
    if ((severe || overload_s_ >= 2.) && level_ < 7) {
      ++level_; ++changes_; overload_s_ = healthy_s_ = 0.; reason_ = severe ? "severe_compute_pressure" : proactive ? "compute_headroom" : "compute_pressure";
    } else if (healthy_s_ >= 8. && level_ > 0) {
      --level_; ++changes_; overload_s_ = healthy_s_ = 0.; reason_ = "recovering";
    } else {
      reason_ = overloaded ? (level_ == 7 ? "minimum_budget" : "pressure_pending") :
                healthy ? "healthy" : (lio_cycles == 0 ? "input_wait" : "holding");
    }
  }

  double vioInterval(double configured) const {
    if (!enabled_ || level_ == 0) return configured;
    static constexpr double extra[] = {0., .05, .1333333333333333, .3, .3, .3, .3, .3};
    // Live 5 Hz becomes 4, 3, then 2 Hz. A slower explicit user limit wins.
    return std::max(configured, std::min(.5, std::max(.1, configured) + extra[level_]));
  }
  double queryFraction() const {
    static constexpr double fractions[] = {1., 1., 1., 1., .75, .5, 1. / 3., .25};
    return enabled_ ? fractions[level_] : 1.;
  }

 private:
  bool enabled_ = false;
  unsigned level_ = 0;
  std::uint64_t changes_ = 0;
  double overload_s_ = 0., healthy_s_ = 0.;
  double compute_ratio_ = 0., ready_backlog_ = 0.;
  const char *reason_ = "disabled";
};

inline std::size_t matchQueryCount(std::size_t size, double fraction, std::size_t budget = 0) {
  // A fraction alone grows with scene density. The optional live budget is
  // applied before the fraction so load reduction remains useful on dense
  // scans. The original scan and map insertion points are never truncated.
  const auto base = budget == 0 ? size : std::min(size, std::max<std::size_t>(1000, budget));
  if (!std::isfinite(fraction) || fraction >= 1.) return base;
  const auto selected = static_cast<std::size_t>(std::ceil(base * std::max(.25, fraction)));
  return std::min(size, std::max<std::size_t>(1000, selected));
}

inline std::size_t matchQueryIndex(std::size_t ordinal, std::size_t selected, std::size_t size) {
  // Spread queries over the complete voxel-ordered cloud, not a prefix or a
  // contiguous time slice. Scan-sized multiplication fits the platform size_t.
  return selected >= size ? ordinal : (ordinal * size + size / 2) / selected;
}

}  // namespace fast_livo_schedule
