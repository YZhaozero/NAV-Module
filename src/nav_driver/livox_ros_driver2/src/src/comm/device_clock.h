#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>

namespace livox_ros {
// One instance per device, shared by point and IMU packets. Delivery time is
// evidence, not sample time. All arguments/results are integer nanoseconds.
class DeviceClock {
 public:
  enum Event { Normal = 0, Anchor = 1, Restart = 2, ModeChange = 3,
               WallStep = 4, Late = 5 };
  struct Result { uint64_t stamp; uint32_t epoch; Event event; bool accept; };
  Result map(uint64_t raw, uint64_t wall, uint64_t mono, uint8_t mode, bool imu) {
    const unsigned stream = imu ? 1 : 0;
    Event event = Normal;
    if (!ready_) event = Anchor;
    else if (mode != mode_) event = ModeChange;
    else if (raw + 1000000000ULL < last_[stream]) event = Restart;
    else if (std::llabs((int64_t(wall)-int64_t(wall_)) -
                       (int64_t(mono)-int64_t(mono_))) > 50000000LL) event = WallStep;
    if (event != Normal) {
      ++epoch_; ready_ = true; mode_ = mode;
      last_[0] = last_[1] = 0;
      frontier_ = window_ = raw;
      wall_origin_ = int64_t(wall)-int64_t(mono);
      offset_ = target_ = minimum_ = int64_t(mono)-int64_t(raw);
    }
    wall_ = wall; mono_ = mono;
    if (raw <= last_[stream])
      return {stamp(raw, mode), epoch_, Late, false};
    last_[stream] = raw;
    if (mode == 0 && raw > frontier_) {
      // Track slow device-clock drift with a one-second lower envelope of
      // arrival delays. Slew at <=100 ppm; a scheduler stall cannot become a
      // fake sample gap. Real missing device samples remain real gaps.
      const int64_t budget = int64_t((raw-frontier_)/10000);
      offset_ += std::max(-budget, std::min(budget, target_-offset_));
      frontier_ = raw;
      minimum_ = std::min(minimum_, int64_t(mono)-int64_t(raw));
      if (raw-window_ >= 1000000000ULL) {
        target_ = minimum_; window_ = raw;
        minimum_ = int64_t(mono)-int64_t(raw);
      }
    }
    return {stamp(raw, mode), epoch_, event, true};
  }
 private:
  uint64_t stamp(uint64_t raw, uint8_t mode) const {
    // TimestampType in comm.h defines PTP/GPTP=1 and GPS=2 as absolute time.
    return mode == 1 || mode == 2 ? raw : uint64_t(int64_t(raw)+offset_+wall_origin_);
  }
  bool ready_ = false;
  uint8_t mode_ = 0;
  uint32_t epoch_ = 0;
  uint64_t last_[2] = {}, frontier_ = 0, window_ = 0, wall_ = 0, mono_ = 0;
  int64_t offset_ = 0, target_ = 0, minimum_ = 0, wall_origin_ = 0;
};
}
