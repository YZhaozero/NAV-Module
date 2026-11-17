#include "comm/ldq.h"
#include <atomic>
#include <thread>
#include <stdexcept>
#include <iostream>
using namespace livox_ros;
int main() {
  LidarDataQueue queue{};
  if (!InitQueue(&queue, 4)) return 1;
  // Cross the 32-bit wrap while producer/consumer run on separate cores.
  queue.rd_idx = UINT32_MAX - 100;
  queue.wr_idx = UINT32_MAX - 100;
  std::atomic<int> errors{0};
  const int count = 20000;
  std::thread writer([&] {
    for (int i = 0; i < count; ++i) {
      std::vector<PointXyzlt> points((i % 97) + 1);
      for (auto &p : points) { p.x = i; p.offset_time = i + 99; }
      PointPacket packet{}; packet.points = points.data(); packet.points_num = points.size();
      while (QueueIsFull(&queue)) std::this_thread::yield();
      QueuePushAny(&queue, reinterpret_cast<uint8_t *>(&packet), i);
    }
  });
  for (int i = 0; i < count; ++i) {
    StoragePacket packet;
    while (!QueuePop(&queue, &packet)) std::this_thread::yield();
    if (packet.base_time != unsigned(i) || packet.points.size() != unsigned((i % 97) + 1)) ++errors;
    for (auto &p : packet.points) if (p.x != i || p.offset_time != unsigned(i + 99)) ++errors;
  }
  writer.join();
  bool empty = QueueIsEmpty(&queue);
  DeInitQueue(&queue);
  std::cout << "queue packets=" << count << " errors=" << errors << " empty=" << empty << '\n';
  return errors == 0 && empty ? 0 : 1;
}
