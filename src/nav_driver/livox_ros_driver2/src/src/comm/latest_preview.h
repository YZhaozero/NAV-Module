#ifndef LIVOX_LATEST_PREVIEW_H_
#define LIVOX_LATEST_PREVIEW_H_

#include <array>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace livox_ros {
// Only the optional display copy may be coalesced. The caller must publish the
// complete measurement stream before submitting here. One pending frame/device,
// one in flight; a slow viewer cannot retain an unbounded history of clouds.
template <typename T, size_t Devices> class LatestPreview {
 public:
  using Publish = std::function<void(size_t, const T&)>;
  explicit LatestPreview(Publish publish)
      : publish_(std::move(publish)), worker_([this] { run(); }) {}
  ~LatestPreview() { stop(); }
  void submit(size_t index, T value) {
    auto pending = std::unique_ptr<T>(new T(std::move(value)));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopped_ || index >= Devices) return;
      pending.swap(pending_[index]);
    }
    ready_.notify_one(); // destroy the replaced frame outside the mutex
  }
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) worker_.join();
  }
 private:
  void run() {
    size_t cursor = 0;
    for (;;) {
      std::unique_ptr<T> frame;
      size_t index = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] {
          if (stopped_) return true;
          for (auto &item : pending_) if (item) return true;
          return false;
        });
        if (stopped_) return;
        for (size_t n = 0; n < Devices; ++n) {
          index = (cursor + n) % Devices;
          if (pending_[index]) {
            frame = std::move(pending_[index]);
            cursor = (index + 1) % Devices;
            break;
          }
        }
      }
      publish_(index, *frame);
    }
  }
  Publish publish_;
  std::array<std::unique_ptr<T>, Devices> pending_{};
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopped_ = false;
  std::thread worker_;
};
} // namespace livox_ros
#endif
