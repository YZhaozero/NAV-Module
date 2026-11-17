#include "patch_similarity.h"

#include <array>
#include <limits>
#include <stdexcept>

static void near(double actual, double expected) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12)
    throw std::runtime_error("Unexpected reference-patch agreement");
}

int main() {
  // Independent scalar correlations: A/B=1, A/C=0, B/C=0. Each reference
  // score must average separate pairs, not cumulative numerator/energies.
  const float a[]{11, 13, 11, 13};
  const float b[]{102, 106, 102, 106};
  const float c[]{7, 7, 9, 9};
  const std::vector<fast_livo_visual::PatchView> views{{a, 1}, {b, 2}, {c, 3}};
  const auto first = fast_livo_visual::patchAgreement(views, 4);
  near(first.means[0], 12.); near(first.means[1], 104.);
  near(first.scores[0], .5); near(first.scores[1], .5); near(first.scores[2], 0.);
  // Re-evaluation with nonzero cached means must remain deterministic.
  const auto second = fast_livo_visual::patchAgreement(views, 4);
  for (std::size_t i = 0; i < views.size(); ++i) near(second.scores[i], first.scores[i]);
  const auto reordered = fast_livo_visual::patchAgreement({views[2], views[0], views[1]}, 4);
  near(reordered.scores[0], first.scores[2]);
  near(reordered.scores[1], first.scores[0]);
  near(reordered.scores[2], first.scores[1]);

  const float inverted[]{9, 7, 9, 7};
  const float flat[]{0, 0, 0, 0};
  const float invalid[]{1, 2, 3, std::numeric_limits<float>::quiet_NaN()};
  const auto mixed = fast_livo_visual::patchAgreement({{a, 1}, {inverted, 2}, {flat, 3}, {invalid, 4}}, 4);
  near(mixed.scores[0], 1.); near(mixed.scores[1], 1.);
  near(mixed.scores[2], 0.); near(mixed.scores[3], 0.);
  const auto same_frame = fast_livo_visual::patchAgreement({{a, 4}, {b, 4}}, 4);
  near(same_frame.scores[0], 0.); near(same_frame.scores[1], 0.);
  const auto single = fast_livo_visual::patchAgreement({{a, 1}}, 4);
  near(single.scores[0], 0.);
  const auto empty = fast_livo_visual::patchAgreement({}, 4);
  if (!empty.scores.empty()) throw std::runtime_error("Empty observations were not empty");
}
