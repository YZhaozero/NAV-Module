#include "patch_samples.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
struct Pixel { float intensity, du, dv; };
// Independent copy of the original per-pixel expressions, not the cache helper.
Pixel original(const std::uint8_t *p, int width, int scale, const std::array<float, 4> &w) {
  Pixel out;
  out.du = .5f *
      ((w[0] * p[scale] + w[1] * p[scale * 2] + w[2] * p[scale * width + scale] + w[3] * p[scale * width + scale * 2]) -
       (w[0] * p[-scale] + w[1] * p[0] + w[2] * p[scale * width - scale] + w[3] * p[scale * width]));
  out.dv = .5f *
      ((w[0] * p[scale * width] + w[1] * p[scale + scale * width] + w[2] * p[width * scale * 2] + w[3] * p[width * scale * 2 + scale]) -
       (w[0] * p[-scale * width] + w[1] * p[-scale * width + scale] + w[2] * p[0] + w[3] * p[scale]));
  out.intensity = w[0] * p[0] + w[1] * p[scale] + w[2] * p[scale * width] + w[3] * p[scale * width + scale];
  return out;
}

void equal(float before, float after) {
  if (std::memcmp(&before, &after, sizeof(float)) != 0)
    throw std::runtime_error("Cached sample changed original float result");
}

std::array<float, 4> weights(float u, float v) {
  return {static_cast<float>((1.0 - u) * (1.0 - v)),
          static_cast<float>(u * (1.0 - v)),
          static_cast<float>((1.0 - u) * v), u * v};
}

void check(int seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> phase(0.f, 1.f);
  fast_livo_visual::PatchSamples cache;
  for (int size : {1, 4, 8, 12, 16}) {
    for (int scale : {1, 2, 4, 8}) {
      // The smallest rectangular source containing the original stencil.
      const int width = (size + 2) * scale + 1;
      std::vector<std::uint8_t> image(width * width);
      const auto *origin = image.data() + scale * width + scale;
      for (int trial = 0; trial < 80; ++trial) {
        for (std::size_t i = 0; i < image.size(); ++i)
          image[i] = trial == 0 ? 0 : trial == 1 ? 255 : trial == 2 ? i % 256 : rng() % 256;
        const auto w = weights(phase(rng), phase(rng));
        cache.fill(origin, width, size, scale, w[0], w[1], w[2], w[3]);
        for (int row = 0; row < size; ++row)
          for (int col = 0; col < size; ++col) {
            const auto expected = original(origin + row * scale * width + col * scale, width, scale, w);
            equal(expected.intensity, cache.value(row, col));
            equal(expected.du, cache.du(row, col));
            equal(expected.dv, cache.dv(row, col));
          }
      }
    }
  }
}

void benchmark() {
  constexpr int width = 848, height = 480, count = 200000;
  std::vector<std::uint8_t> image(width * height);
  std::mt19937 rng(2946);
  for (auto &pixel : image) pixel = rng() % 256;
  std::vector<int> origins;
  std::vector<std::array<float, 4>> phases;
  std::uniform_real_distribution<float> phase(0.f, 1.f);
  for (int i = 0; i < count; ++i) {
    origins.push_back((16 + rng() % 440) * width + 16 + rng() % 800);
    phases.push_back(weights(phase(rng), phase(rng)));
  }
  fast_livo_visual::PatchSamples cache;
  std::array<double, 2> times{}, sums{};
  for (int implementation = 0; implementation < 2; ++implementation) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < count; ++i) {
      const auto *origin = image.data() + origins[i];
      const auto &w = phases[i];
      if (implementation) cache.fill(origin, width, 8, 1, w[0], w[1], w[2], w[3]);
      for (int row = 0; row < 8; ++row)
        for (int col = 0; col < 8; ++col) {
          const auto result = implementation ? Pixel{cache.value(row, col), cache.du(row, col), cache.dv(row, col)} :
              original(origin + row * width + col, width, 1, w);
          sums[implementation] += result.intensity + result.du + result.dv;
        }
    }
    times[implementation] = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  }
  if (sums[0] != sums[1]) throw std::runtime_error("Benchmark checksum differs");
  std::cout << "original_s=" << times[0] << " cached_s=" << times[1]
            << " speedup=" << times[0] / times[1] << " checksum=" << sums[0] << '\n';
}
}  // namespace

int main(int argc, char **argv) {
  // Exercise independent worker scratch concurrently as in the VIO OpenMP loop.
  #pragma omp parallel for num_threads(4)
  for (int worker = 0; worker < 4; ++worker) check(487 + worker);
  std::cout << "Patch values and gradients are bit-identical for 6400 patches\n";
  if (argc > 1 && std::string(argv[1]) == "--benchmark") benchmark();
}
