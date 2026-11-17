#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fast_livo_visual {

struct PatchView {
  const float *pixels;
  int frame_id;
};

struct PatchAgreement {
  std::vector<double> means;
  std::vector<double> scores;
};

// Mean absolute normalized correlation with the other observations of a point.
// Patches are immutable. Compute centering/energy once, and each symmetric
// pair once. A textureless/invalid patch supplies no photometric evidence.
inline PatchAgreement patchAgreement(const std::vector<PatchView> &patches,
                                     std::size_t pixel_count) {
  const std::size_t n = patches.size();
  PatchAgreement result{std::vector<double>(n, 0.), std::vector<double>(n, 0.)};
  if (!pixel_count) return result;
  std::vector<double> centered(n * pixel_count, 0.);
  std::vector<double> energy(n, 0.);
  std::vector<std::size_t> counts(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (!patches[i].pixels) continue;
    double sum = 0.;
    for (std::size_t p = 0; p < pixel_count; ++p) sum += patches[i].pixels[p];
    if (!std::isfinite(sum)) continue;
    result.means[i] = sum / pixel_count;
    for (std::size_t p = 0; p < pixel_count; ++p) {
      const double value = patches[i].pixels[p] - result.means[i];
      centered[i * pixel_count + p] = value;
      energy[i] += value * value;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!(energy[i] > 0.) || !std::isfinite(energy[i])) continue;
    for (std::size_t j = i + 1; j < n; ++j) {
      if (patches[i].frame_id == patches[j].frame_id ||
          !(energy[j] > 0.) || !std::isfinite(energy[j])) continue;
      double dot = 0.;
      for (std::size_t p = 0; p < pixel_count; ++p)
        dot += centered[i * pixel_count + p] * centered[j * pixel_count + p];
      const double score = std::min(1., std::abs(dot / std::sqrt(energy[i] * energy[j])));
      result.scores[i] += score;
      result.scores[j] += score;
      ++counts[i];
      ++counts[j];
    }
  }
  for (std::size_t i = 0; i < n; ++i)
    if (counts[i]) result.scores[i] /= counts[i];
  return result;
}

}  // namespace fast_livo_visual
