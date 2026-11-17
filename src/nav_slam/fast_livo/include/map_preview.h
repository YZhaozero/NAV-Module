#pragma once
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include "voxel_key.h"
#include <stdexcept>
#include <tuple>

// Rendering budget only. Coarsen the WHOLE scene instead of evicting old areas.
// The independently persisted archive never goes through this representation.
struct PreviewPoint {
  float x, y, z;
  uint8_t r = 160, g = 175, b = 185;
  bool colored = false;
};
class MapPreview {
 public:
  MapPreview(double leaf = .05, size_t limit = 120000) : base_(leaf), leaf_(leaf), limit_(limit) {
    if (!std::isfinite(leaf) || leaf <= 0 || limit < 1) throw std::invalid_argument("Invalid preview budget");
  }
  using Key = VoxelKey;
  using Points = std::unordered_map<Key, PreviewPoint, VoxelKeyHash>;
  void add(PreviewPoint p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x)>1e6 || std::abs(p.y)>1e6 || std::abs(p.z)>1e6) return;
    put(points_, p);
    while (points_.size() > limit_) {
      leaf_ *= 2;
      Points reduced;
      reduced.reserve(points_.size()/2);
      for (const auto &entry : points_) put(reduced, entry.second);
      points_.swap(reduced);
    }
  }
  const Points &points() const { return points_; }
  double leaf() const { return leaf_; }
  void clear() { points_.clear(); leaf_ = base_; }
 private:
  void put(Points &target, PreviewPoint p) {
    // Round-to-nearest avoids a permanent eight-cell split at the origin.
    const Key key{int64_t(std::floor(p.x/leaf_+.5)), int64_t(std::floor(p.y/leaf_+.5)), int64_t(std::floor(p.z/leaf_+.5))};
    auto [found, inserted] = target.try_emplace(key,p);
    if (inserted) return;
    if (found->second.colored && !p.colored) {
      p.r=found->second.r; p.g=found->second.g; p.b=found->second.b; p.colored=true;
    }
    found->second=p;
  }
  double base_, leaf_;
  size_t limit_;
  Points points_;
};
