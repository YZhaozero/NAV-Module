#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

// Debug state output must not stall IMU/LIO while the recorder flushes the SSD.
// The worker owns all file operations; only this bounded text queue is shared.
class AsyncStateLog {
public:
  using Sink = std::function<void(unsigned, const std::string &)>;
  explicit AsyncStateLog(const std::array<std::string, 2> &paths)
      : sink_([this](unsigned channel, const std::string &line) {
          streams_[channel] << line;
          if (!streams_[channel]) ++errors_;
        }), worker_([this, paths] {
          for (unsigned i=0; i<paths.size(); ++i) {
            if (paths[i].empty()) continue;
            std::error_code error;
            const auto parent = std::filesystem::path(paths[i]).parent_path();
            if (!parent.empty()) std::filesystem::create_directories(parent, error);
            if (error) { ++errors_; continue; }
            // A logger reopened in the same process must preserve earlier
            // evidence. A fresh mapper process has its own session directory.
            streams_[i].open(paths[i], std::ios::out | std::ios::app);
            if (!streams_[i]) ++errors_;
          }
          drain();
          for (auto &stream : streams_)
            if (stream.is_open()) { stream.flush(); if (!stream) ++errors_; }
        }) {}
  // Also permits a deliberately slow sink in the regression test.
  explicit AsyncStateLog(Sink sink, size_t limit=512*1024)
      : limit_(limit), sink_(std::move(sink)), worker_([this] { drain(); }) {}
  ~AsyncStateLog() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
    wake_.notify_one(); worker_.join();
  }
  bool append(unsigned channel, std::string line) {
    if (channel>=2) return false;
    { std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || line.size()>limit_-bytes_) { ++dropped_; return false; }
      bytes_+=line.size(); queue_.push_back({channel,std::move(line)});
    }
    wake_.notify_one(); return true;
  }
  size_t dropped() const { return dropped_.load(); }
  size_t errors() const { return errors_.load(); }
private:
  void drain() {
    for (;;) {
      std::pair<unsigned,std::string> row;
      { std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty()) return;
        row=std::move(queue_.front()); queue_.pop_front(); bytes_-=row.second.size();
      }
      try { sink_(row.first,row.second); } catch (...) { ++errors_; }
    }
  }
  std::array<std::ofstream,2> streams_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<std::pair<unsigned,std::string>> queue_;
  size_t bytes_=0, limit_=512*1024;
  bool stopping_=false;
  std::atomic<size_t> dropped_{0}, errors_{0};
  Sink sink_;
  std::thread worker_;
};
