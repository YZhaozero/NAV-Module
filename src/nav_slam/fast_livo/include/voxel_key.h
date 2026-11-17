#pragma once
#include <cstdint>
#include <tuple>
#include <functional>

using VoxelKey = std::tuple<int64_t, int64_t, int64_t>;
struct VoxelKeyHash {
  size_t operator()(const VoxelKey &key) const noexcept {
    size_t value = std::hash<int64_t>{}(std::get<0>(key));
    for (auto item : {std::get<1>(key), std::get<2>(key)})
      value ^= std::hash<int64_t>{}(item) + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2);
    return value;
  }
};
