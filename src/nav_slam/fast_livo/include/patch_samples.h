#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fast_livo_visual {

// Cache the bilinear samples shared by a patch's intensities and central
// differences. The original formulas visit the same sample up to five times.
// Keep float operations and their order identical to the uncached formulas.
class PatchSamples {
 public:
  void fill(const std::uint8_t *origin, std::ptrdiff_t stride, int size, int scale,
            float top_left, float top_right, float bottom_left, float bottom_right) {
    pitch_ = size + 2;
    values_.resize(static_cast<std::size_t>(pitch_) * pitch_);
    const std::ptrdiff_t row_step = stride * scale;
    for (int row = -1; row <= size; ++row) {
      const auto *pixel = origin + row * row_step - scale;
      for (int col = -1; col <= size; ++col, pixel += scale) {
        // These corners are never used by a central difference. Avoid reading
        // beyond the original sampling footprint at image boundaries.
        if ((row == -1 || row == size) && (col == -1 || col == size)) continue;
        values_[(row + 1) * pitch_ + col + 1] =
            top_left * pixel[0] + top_right * pixel[scale] +
            bottom_left * pixel[row_step] + bottom_right * pixel[row_step + scale];
      }
    }
  }

  float value(int row, int col) const { return values_[(row + 1) * pitch_ + col + 1]; }
  float du(int row, int col) const { return .5f * (value(row, col + 1) - value(row, col - 1)); }
  float dv(int row, int col) const { return .5f * (value(row + 1, col) - value(row - 1, col)); }

 private:
  int pitch_ = 0;
  std::vector<float> values_;
};

}  // namespace fast_livo_visual
