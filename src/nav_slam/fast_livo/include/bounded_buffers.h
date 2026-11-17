#pragma once

#include <algorithm>
#include <cstddef>
#include <stdexcept>

namespace fast_livo_memory {
constexpr std::size_t kImages = 8;
constexpr std::size_t kLidar = 30;
constexpr std::size_t kImu = 4000;

// Payloads and timestamps always move together, including the pending LIO scan.
template <class Queue, class Times>
std::size_t trimPaired(Queue &queue, Times &times, std::size_t limit) {
  if (!limit || queue.size() != times.size())
    throw std::logic_error("invalid paired sensor buffer");
  std::size_t dropped = 0;
  while (queue.size() > limit) {
    queue.pop_front();
    times.pop_front();
    ++dropped;
  }
  return dropped;
}

// The sink must persist a chunk successfully before the buffer is released.
// A single large input is split too: the saving buffer never exceeds limit.
template <class Cloud, class Sink>
void appendCloud(Cloud &buffer, const Cloud &input, std::size_t limit, Sink sink) {
  if (!limit || buffer.size() > limit)
    throw std::logic_error("invalid point cloud saving limit");
  std::size_t offset = 0;
  while (offset < input.size()) {
    if (buffer.size() == limit) {
      sink(buffer);
      Cloud().swap(buffer);
    }
    const auto count = std::min(limit - buffer.size(), input.size() - offset);
    buffer.insert(buffer.end(), input.begin() + offset, input.begin() + offset + count);
    offset += count;
    if (buffer.size() == limit) {
      sink(buffer);
      Cloud().swap(buffer);
    }
  }
}
} // namespace fast_livo_memory
