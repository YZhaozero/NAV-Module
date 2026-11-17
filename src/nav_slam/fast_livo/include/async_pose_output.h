#pragma once
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <nav_msgs/msg/odometry.hpp>

// Pose history is measured output, not a replaceable preview. Preserve order
// through a bounded transport stall, and explicitly report overflow/failure.
// The sink must own its publishers: shutdown may outlive the mapper if DDS is
// stuck. No callback may capture the mapper or borrowed references.
class AsyncPoseOutput {
 public:
  using Pose = nav_msgs::msg::Odometry;
  using Sink = std::function<void(const Pose &)>;
  struct Stats {
    size_t submitted=0, completed=0, rejected=0, pending=0;
    bool failed=false, active=false;
    double stamp=-1., last_publish_s=0.;
  };
  explicit AsyncPoseOutput(Sink sink, size_t capacity=2048)
      : state_(std::make_shared<State>(std::move(sink), capacity)),
        thread_([s=state_] { work(s); }) {}
  ~AsyncPoseOutput() { stop(); }
  AsyncPoseOutput(const AsyncPoseOutput &) = delete;
  AsyncPoseOutput &operator=(const AsyncPoseOutput &) = delete;
  bool submit(Pose pose) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->stopping || state_->stats.failed || state_->queue.size() >= state_->capacity) {
      ++state_->stats.rejected;
      // Latch: a missing pose must not silently become a continuous trajectory.
      state_->stats.failed = true;
      return false;
    }
    state_->queue.push_back(std::move(pose)); ++state_->stats.submitted;
    state_->wake.notify_one(); return true;
  }
  Stats stats() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto result=state_->stats; result.pending=state_->queue.size(); return result;
  }
  // The caller can save the map and shut down even if a transport never returns.
  // Pending poses are drained when possible; a timeout is reported as false.
  bool stop(std::chrono::milliseconds timeout=std::chrono::milliseconds(2000)) {
    bool done;
    {
      std::unique_lock<std::mutex> lock(state_->mutex);
      if (!thread_.joinable()) return state_->done;
      state_->stopping=true; state_->wake.notify_all();
      done=state_->finished.wait_for(lock, timeout, [&] { return state_->done; });
    }
    if (thread_.joinable()) { if (done) thread_.join(); else thread_.detach(); }
    return done;
  }
 private:
  struct State {
    State(Sink f, size_t n): sink(std::move(f)), capacity(n) {}
    Sink sink; const size_t capacity;
    std::mutex mutex; std::condition_variable wake, finished;
    std::deque<Pose> queue; Stats stats;
    bool stopping=false, done=false;
  };
  static void work(const std::shared_ptr<State> &s) {
    for (;;) {
      Pose pose;
      {
        std::unique_lock<std::mutex> lock(s->mutex);
        s->wake.wait(lock,[&] { return s->stopping || !s->queue.empty(); });
        if (s->queue.empty()) break;
        pose=std::move(s->queue.front()); s->queue.pop_front(); s->stats.active=true;
      }
      const auto start=std::chrono::steady_clock::now(); bool failed=false;
      try { s->sink(pose); } catch (...) { failed=true; }
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->stats.active=false;
        s->stats.last_publish_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        if (failed) {
          s->stats.failed=true; s->stats.rejected+=1+s->queue.size(); s->queue.clear(); break;
        }
        ++s->stats.completed;
        s->stats.stamp=double(pose.header.stamp.sec)+pose.header.stamp.nanosec*1e-9;
      }
    }
    { std::lock_guard<std::mutex> lock(s->mutex); s->done=true; }
    s->finished.notify_all();
  }
  std::shared_ptr<State> state_; std::thread thread_;
};
