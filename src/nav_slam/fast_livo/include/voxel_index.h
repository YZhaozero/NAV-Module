#pragma once

#include <cmath>
#include <cstdint>

namespace fast_livo_voxel {

// A voxel covers [index * size, (index + 1) * size). Use the same rule
// for insertion and lookup, especially to the negative side of the origin.
// Callers supply finite coordinates and a positive, finite voxel size.
inline std::int64_t voxelIndex(double coordinate, double voxel_size) {
  return static_cast<std::int64_t>(std::floor(coordinate / voxel_size));
}

}  // namespace fast_livo_voxel
