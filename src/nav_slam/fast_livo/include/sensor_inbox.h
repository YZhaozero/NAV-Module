#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>

// The ROS receive thread only transfers shared message ownership. Preprocessing,
// timestamp-gap recovery and all estimator state remain on the estimator thread.
// Per-stream budgets prevent a burst of images from evicting IMU measurements.
class SensorInbox {
public:
  enum Stream { Lidar = 0, Imu = 1, Image = 2 };
  struct Limit { std::size_t count, bytes; };
  struct Stats {
    std::array<std::size_t, 3> count{}, bytes{};
    std::array<std::uint64_t, 3> received{}, dropped{};
    std::array<double, 3> stamp{}, received_at{}, interval{}, steady{};
    std::array<bool, 3> backwards{};
  };
  explicit SensorInbox(std::array<Limit, 3> limits = {{
      {128, 32 * 1024 * 1024}, {4000, 2 * 1024 * 1024}, {16, 32 * 1024 * 1024}}})
      : limits_(limits) {}

  void push(Stream stream, std::size_t bytes, std::function<void()> consume,
            double stamp = 0., double received_at = 0.) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) return;
    auto &q = queues_[stream];
    auto &used = stats_.bytes[stream];
    const auto limit = limits_[stream];
    const double steady = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    stats_.interval[stream] = stats_.received[stream] ? steady - stats_.steady[stream] : 0.;
    stats_.backwards[stream] = stats_.received[stream] && stamp < stats_.stamp[stream];
    stats_.steady[stream] = steady;
    stats_.stamp[stream] = stamp; stats_.received_at[stream] = received_at;
    ++stats_.received[stream];
    if (stream == Imu && (q.size() >= limit.count || bytes > limit.bytes - used)) {
      failure_ = std::make_exception_ptr(std::runtime_error(
          "IMU receive backlog exceeded bounded budget; restart localization"));
      return;
    }
    if (bytes > limit.bytes || !limit.count) {
      ++stats_.dropped[stream];
      return;
    }
    while (!q.empty() && (q.size() >= limit.count || bytes > limit.bytes - used)) {
      used -= q.front().bytes;
      q.pop_front();
      ++stats_.dropped[stream];
    }
    q.push_back({++sequence_, bytes, std::move(consume)});
    used += bytes;
    stats_.count[stream] = q.size();
  }

  // Preserve callback arrival order across streams; never execute work under
  // the queue lock. A slow consumer cannot lock out the receive thread.
  std::function<void()> take() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    std::size_t selected = 3;
    auto sequence = std::numeric_limits<std::uint64_t>::max();
    for (std::size_t i = 0; i < 3; ++i)
      if (!queues_[i].empty() && queues_[i].front().sequence < sequence) {
        selected = i; sequence = queues_[i].front().sequence;
      }
    if (selected == 3) return {};
    auto &q = queues_[selected];
    auto work = std::move(q.front().consume);
    stats_.bytes[selected] -= q.front().bytes;
    q.pop_front(); stats_.count[selected] = q.size();
    return work;
  }
  Stats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }
  void fail(std::exception_ptr error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!failure_) failure_ = error;
  }
private:
  struct Item { std::uint64_t sequence; std::size_t bytes; std::function<void()> consume; };
  const std::array<Limit, 3> limits_;
  mutable std::mutex mutex_;
  std::array<std::deque<Item>, 3> queues_;
  Stats stats_;
  std::uint64_t sequence_ = 0;
  std::exception_ptr failure_;
};
