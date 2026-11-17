#include "voxel_index.h"

#include <limits>
#include <stdexcept>
#include <unordered_map>

static void check(double coordinate, double size, std::int64_t expected) {
  if (fast_livo_voxel::voxelIndex(coordinate, size) != expected)
    throw std::runtime_error("Incorrect voxel index");
}

int main() {
  const double inf = std::numeric_limits<double>::infinity();
  check(0., .5, 0);
  check(-0., .5, 0);
  check(.1, .5, 0);
  check(-.1, .5, -1);
  check(.5, .5, 1);
  check(-.5, .5, -1);
  check(1., .5, 2);
  check(-1., .5, -2);
  check(std::nextafter(.5, -inf), .5, 0);
  check(std::nextafter(.5, inf), .5, 1);
  check(std::nextafter(-.5, -inf), .5, -2);
  check(std::nextafter(-.5, inf), .5, -1);
  check(std::nextafter(-1., -inf), .5, -3);
  check(std::nextafter(-1., inf), .5, -2);
  check(-2.1, 2., -2);
  check(-2., 2., -1);

  // The prior visual-map implementation inserted -0.1 m into cell -1,
  // but searched cell -2 after applying a second negative-floor adjustment.
  std::unordered_map<std::int64_t, int> inserted;
  inserted[fast_livo_voxel::voxelIndex(-.1, .5)] = 42;
  if (inserted.at(fast_livo_voxel::voxelIndex(-.2, .5)) != 42)
    throw std::runtime_error("Points in the same negative voxel did not match");
  if (inserted.count(fast_livo_voxel::voxelIndex(-.6, .5)))
    throw std::runtime_error("Adjacent negative voxels were merged");
}
