#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <string>

namespace livox_ros {
// Little endian on supported Jetson/x86 hosts. Version 1, 80 bytes per record.
#pragma pack(push, 1)
struct PacketEvidenceRecord {
  uint64_t wall_ns, mono_ns, device_ns, mapped_ns;
  uint32_t handle, epoch;
  uint16_t sequence;
  uint8_t type, clock_type, event, accepted;
  uint16_t reserved = 0;
  uint64_t dropped_before = 0;
  uint8_t imu_payload[24] = {};
};
#pragma pack(pop)
static_assert(sizeof(PacketEvidenceRecord) == 80, "evidence schema");

class PacketEvidence {
 public:
  PacketEvidence();
  ~PacketEvidence();
  void close();
  void push(PacketEvidenceRecord record);
 private:
  void run();
  std::string directory_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<PacketEvidenceRecord> queue_;
  bool closing_ = false;
  uint64_t dropped_ = 0, accepted_ = 0;
  std::thread worker_;
};
}
