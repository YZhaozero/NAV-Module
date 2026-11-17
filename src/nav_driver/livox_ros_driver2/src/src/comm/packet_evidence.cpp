#include "packet_evidence.h"
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <vector>

namespace livox_ros {
PacketEvidence::PacketEvidence() {
  const char* root = std::getenv("NAV_EVIDENCE_DIR");
  const char* home = std::getenv("HOME");
  directory_ = root ? root : std::string(home ? home : "/tmp")+"/.local/state/nav/evidence";
  directory_ += "/livox-"+std::to_string(::getpid())+"-"+std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count());
  worker_ = std::thread(&PacketEvidence::run, this);
}
PacketEvidence::~PacketEvidence() {
  close();
}
void PacketEvidence::close() {
  { std::lock_guard<std::mutex> lock(mutex_); closing_ = true; }
  ready_.notify_one(); if(worker_.joinable()) worker_.join();
}
void PacketEvidence::push(PacketEvidenceRecord record) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.size() >= 8192 || closing_) { ++dropped_; return; }
  record.dropped_before = dropped_; ++accepted_;
  queue_.push_back(record); ready_.notify_one();
}
void PacketEvidence::run() {
  for (size_t pos = 1; pos <= directory_.size(); ++pos)
    if (pos == directory_.size() || directory_[pos] == '/')
      ::mkdir(directory_.substr(0,pos).c_str(), 0700);
  std::cout << "LIVOX_EVIDENCE " << directory_ << std::endl;
  int fd = -1; unsigned segment = 0; size_t size = 0;
  uint64_t written = 0, failed = 0;
  std::string error;
  auto checked = std::chrono::steady_clock::time_point{};
  auto reported = checked;
  bool disk_ok = true, done = false;
  while (!done) {
    std::vector<PacketEvidenceRecord> batch;
    uint64_t dropped, accepted; size_t queued;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait_for(lock, std::chrono::milliseconds(200), [&]{return closing_ || !queue_.empty();});
      for (size_t n = 0; n < 256 && !queue_.empty(); ++n) {
        batch.push_back(queue_.front()); queue_.pop_front();
      }
      done = closing_ && queue_.empty(); dropped = dropped_; accepted = accepted_; queued = queue_.size();
    }
    auto now = std::chrono::steady_clock::now();
    if (now-checked > std::chrono::seconds(1)) {
      struct statvfs space;
      disk_ok = ::statvfs(directory_.c_str(), &space) == 0 &&
          uint64_t(space.f_bavail)*space.f_frsize > 256ULL*1024*1024;
      checked = now;
      if (!disk_ok) error = "disk_unavailable_or_below_256MiB";
    }
    if (!batch.empty() && disk_ok) {
      if (fd < 0 || size >= 16*1024*1024) {
        if (fd >= 0) { ::fsync(fd); ::close(fd); }
        char suffix[48]; std::snprintf(suffix,sizeof(suffix),"/packets-%06u.bin",segment++);
        fd = ::open((directory_+suffix).c_str(),O_CREAT|O_EXCL|O_WRONLY,0600); size = 0;
      }
      const size_t bytes = batch.size()*sizeof(PacketEvidenceRecord);
      size_t offset = 0;
      while (fd >= 0 && offset < bytes) {
        ssize_t n = ::write(fd,reinterpret_cast<const char*>(batch.data())+offset,bytes-offset);
        if (n <= 0) break;
        offset += n;
      }
      size += offset; written += offset/sizeof(PacketEvidenceRecord);
      failed += batch.size()-offset/sizeof(PacketEvidenceRecord);
      if (offset != bytes) { error = "write_failed"; if(fd>=0) ::close(fd); fd=-1; }
    } else failed += batch.size();
    if (done || now-reported >= std::chrono::seconds(1)) {
      if (fd >= 0 && ::fsync(fd) != 0) error = "fsync_failed";
      const std::string status = "{\"schema_version\":1,\"record_bytes\":80,\"queue_limit_bytes\":655360,\"accepted\":"+
        std::to_string(accepted)+",\"written\":"+std::to_string(written)+",\"dropped\":"+
        std::to_string(dropped)+",\"write_failed_records\":"+std::to_string(failed)+
        ",\"queued\":"+std::to_string(queued)+",\"closed\":"+(done?"true":"false")+
        ",\"error\":\""+error+"\"}\n";
      const std::string temp = directory_+"/status.json.tmp";
      int out = ::open(temp.c_str(),O_CREAT|O_TRUNC|O_WRONLY,0600);
      if (out>=0) {
        const ssize_t count = ::write(out,status.data(),status.size());
        ::close(out);
        if(count == static_cast<ssize_t>(status.size())) ::rename(temp.c_str(),(directory_+"/status.json").c_str());
        else std::cerr << "LIVOX_EVIDENCE status_write_failed" << std::endl;
      }
      reported=now;
    }
  }
  if(fd>=0) ::close(fd);
}
}
