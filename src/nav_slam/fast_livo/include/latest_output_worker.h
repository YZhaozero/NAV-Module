#pragma once

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace fast_livo_output {

// For replaceable display output, never for measured-data archival. A slow sink
// retains one active item and one newest pending item, without blocking submit.
// The callback must not retain every input or the caller can defeat this bound.
template <typename T> class LatestOutputWorker {
 public:
  using Item = std::shared_ptr<const T>;
  using Callback = std::function<void(Item)>;
  struct Stats {
    std::uint64_t submitted = 0;
    std::uint64_t completed = 0;
    std::uint64_t replaced = 0;
    std::uint64_t discarded = 0;
    bool accepting = true;
    bool active = false;
    bool pending = false;
    bool failed = false;
  };

  explicit LatestOutputWorker(Callback callback)
      : state_(std::make_shared<State>(std::move(callback))) {
    if (!state_->callback) throw std::invalid_argument("Output callback is empty");
    // The thread owns state, rather than this, so even self-destruction from a
    // callback does not invalidate the queue while its callback unwinds.
    worker_ = std::thread([state = state_] { work(state); });
    worker_id_ = worker_.get_id();
  }
  LatestOutputWorker(const LatestOutputWorker &) = delete;
  LatestOutputWorker &operator=(const LatestOutputWorker &) = delete;

  ~LatestOutputWorker() noexcept {
    stop();
    // stop() requested from the callback cannot join itself. The thread's
    // shared state remains alive until work() finishes its current callback.
    if (worker_.joinable()) worker_.detach();
  }

  // False means shutdown has begun. A previous callback failure is rethrown,
  // including after stop(), so an output failure cannot silently disappear.
  bool submit(Item item) {
    if (!item) throw std::invalid_argument("Output item is empty");
    Item replaced;
    std::exception_ptr failure;
    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      failure = state_->failure;
      if (!failure && state_->stats.accepting) {
        replaced = std::move(state_->pending);
        if (replaced) ++state_->stats.replaced;
        state_->pending = std::move(item);
        ++state_->stats.submitted;
        accepted = true;
      }
    }
    if (accepted) state_->wake.notify_one();
    // A payload may have a large buffer or a reentrant deleter: never destroy
    // it while holding the queue mutex.
    replaced.reset();
    if (failure) std::rethrow_exception(failure);
    return accepted;
  }

  // Concurrent stop/submit and concurrent external stop calls are supported.
  // The active callback finishes; drain_latest additionally runs the newest
  // pending item. Calling from the callback requests shutdown without joining.
  // Other callers and destruction must not race destruction of this object.
  void stop(bool drain_latest = true) noexcept {
    Item discarded;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->stats.accepting = false;
      state_->stopping = true;
      if (!drain_latest) {
        discarded = std::move(state_->pending);
        if (discarded) ++state_->stats.discarded;
      }
    }
    state_->wake.notify_all();
    discarded.reset();
    if (std::this_thread::get_id() == worker_id_) return;
    // Only joining needs serialization. Never hold the queue mutex or this
    // mutex from a callback; an external join may be waiting for that callback.
    std::lock_guard<std::mutex> join_lock(join_mutex_);
    if (worker_.joinable()) worker_.join();
  }

  void rethrowIfFailed() const {
    std::exception_ptr failure;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      failure = state_->failure;
    }
    if (failure) std::rethrow_exception(failure);
  }

  Stats stats() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    Stats result = state_->stats;
    result.pending = bool(state_->pending);
    result.failed = bool(state_->failure);
    return result;
  }

 private:
  struct State {
    explicit State(Callback cb) : callback(std::move(cb)) {}
    std::mutex mutex;
    std::condition_variable wake;
    Callback callback;
    Item pending;
    Stats stats;
    bool stopping = false;
    std::exception_ptr failure;
  };

  static void work(const std::shared_ptr<State> &state) noexcept {
    for (;;) {
      Item item;
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        state->wake.wait(lock, [&] { return state->stopping || bool(state->pending); });
        if (!state->pending) return;
        item = std::move(state->pending);
        state->stats.active = true;
      }
      std::exception_ptr failure;
      try { state->callback(item); }
      catch (...) { failure = std::current_exception(); }
      item.reset();
      Item discarded;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->stats.active = false;
        if (failure) {
          state->failure = failure;
          state->stats.accepting = false;
          state->stopping = true;
          discarded = std::move(state->pending);
          if (discarded) ++state->stats.discarded;
        } else {
          ++state->stats.completed;
        }
      }
      discarded.reset();
      if (failure) return;
    }
  }

  std::shared_ptr<State> state_;
  std::thread worker_;
  std::thread::id worker_id_;
  std::mutex join_mutex_;
};

}  // namespace fast_livo_output
