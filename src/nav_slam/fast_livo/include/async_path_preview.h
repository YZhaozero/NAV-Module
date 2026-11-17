#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace fast_livo_preview {

// Only the live visualization is asynchronous. Original odometry, map storage,
// and recorded poses remain the caller's responsibility and are never dropped.
class AsyncPathPreview {
public:
  using Pose = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;
  using Publish = std::function<void(Path::UniquePtr)>;
  using HasSubscribers = std::function<bool()>;
  struct Options {
    std::chrono::steady_clock::duration period = std::chrono::seconds(1);
    std::size_t history_limit = 10000;
    std::size_t prune_count = 1000;
    std::size_t pending_limit = 10000;
  };
  struct Stats {
    std::uint64_t accepted = 0, dropped_preview_poses = 0, published = 0;
    std::size_t pending = 0, peak_pending = 0;
  };

  AsyncPathPreview(Publish publish, HasSubscribers subscribers)
      : AsyncPathPreview(std::move(publish), std::move(subscribers), Options{}) {}

  AsyncPathPreview(Publish publish, HasSubscribers subscribers, Options options)
      : publish_(std::move(publish)), subscribers_(std::move(subscribers)), options_(options) {
    if (!publish_ || !subscribers_ || options_.period <= Clock::duration::zero() ||
        !options_.history_limit || !options_.prune_count ||
        options_.prune_count > options_.history_limit || !options_.pending_limit)
      throw std::invalid_argument("Invalid asynchronous path preview options");
    worker_ = std::thread([this] { run(); });
  }
  AsyncPathPreview(const AsyncPathPreview &) = delete;
  AsyncPathPreview &operator=(const AsyncPathPreview &) = delete;
  ~AsyncPathPreview() { stop(); }

  // Single producer. Neither history copying nor publisher execution holds this
  // mutex. Overflow drops one oldest preview pose, never a sensor measurement.
  bool append(const Pose &pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (stopping_) return false;
    if (has_previous_ && (pose.header.frame_id != previous_.frame_id ||
                         before(pose.header.stamp, previous_.stamp))) ++epoch_;
    previous_ = pose.header;
    has_previous_ = true;
    if (pending_.size() == options_.pending_limit) {
      pending_.pop_front();
      ++stats_.dropped_preview_poses;
    }
    pending_.push_back({pose, ++sequence_, epoch_.load()});
    ++stats_.accepted;
    stats_.pending = pending_.size();
    stats_.peak_pending = std::max(stats_.peak_pending, stats_.pending);
    return true;
  }

  Stats stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }
  void rethrowIfFailed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
  }
  // Lifecycle calls belong to the owning thread. A publisher callback already
  // executing must return before join completes; append never waits for it.
  void stop() noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    wake_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Item { Pose pose; std::uint64_t sequence, epoch; };
  static bool before(const builtin_interfaces::msg::Time &a,
                     const builtin_interfaces::msg::Time &b) {
    return a.sec < b.sec || (a.sec == b.sec && a.nanosec < b.nanosec);
  }
  void run() noexcept {
    try {
      std::deque<Item> history;
      auto next = Clock::now();
      for (;;) {
        std::deque<Item> batch;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait_until(lock, next, [this] { return stopping_; });
          if (stopping_) return;
          batch.swap(pending_);  // O(1); no pose copies under the producer lock.
          stats_.pending = 0;
        }
        for (auto &item : batch) {
          // A session change or an overflow must not connect two disconnected
          // pieces of a trajectory with an invented straight segment.
          if (!history.empty() && (item.epoch != history.back().epoch ||
                                   item.sequence != history.back().sequence + 1)) history.clear();
          history.push_back(std::move(item));
          if (history.size() > options_.history_limit)
            for (std::size_t i = 0; i < options_.prune_count; ++i) history.pop_front();
        }
        const bool subscribed = subscribers_();
        // Refresh even an unchanged path: a new subscriber can arrive while
        // another remains connected, without any observable bool transition.
        if (subscribed && !history.empty() &&
            history.back().epoch == epoch_.load()) {
          auto path = std::make_unique<Path>();
          path->header = history.back().pose.header;  // Measurement time, not wall time.
          path->poses.reserve(history.size());
          for (const auto &item : history) path->poses.push_back(item.pose);
          if (history.back().epoch == epoch_.load()) {
            publish_(std::move(path));
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.published;
          }
        }
        // Schedule from completion, never catch up with publication bursts.
        next = Clock::now() + options_.period;
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      failure_ = std::current_exception();
    }
  }

  Publish publish_;
  HasSubscribers subscribers_;
  const Options options_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Item> pending_;
  std_msgs::msg::Header previous_;
  std::atomic<std::uint64_t> epoch_{0};
  std::uint64_t sequence_ = 0;
  Stats stats_;
  std::exception_ptr failure_;
  bool has_previous_ = false, stopping_ = false;
  std::thread worker_;
};

}  // namespace fast_livo_preview
