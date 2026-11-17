#include "adaptive_load.h"
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

static void check(bool value, int line) {
  if (!value) throw std::runtime_error("adaptive load regression line " + std::to_string(line));
}
#define require(x) check((x), __LINE__)

int main() {
  using namespace fast_livo_schedule;
  AdaptiveLoad load;
  for (int i = 0; i < 30; ++i) load.observe(1., .95, 2., 10);
  require(load.level() == 0 && load.vioInterval(.2) == .2 && load.queryFraction() == 1.);
  load.enable(true);
  // Old input or blocked output may leave a large backlog, without spending
  // that second computing. Neither should degrade the visual observations.
  for (int i = 0; i < 60; ++i) load.observe(1., .1, 2., 10);
  require(load.level() == 0);
  for (int i = 0; i < 60; ++i) load.observe(1., .95, 0., 10);
  require(load.level() == 7); // Compute pressure acts before backlog appears.
  load.enable(false); load.enable(true);
  // One burst followed by a healthy window is deliberately insufficient.
  for (int i = 0; i < 30; ++i) {
    load.observe(1., .95, .3, 10); load.observe(1., .2, .02, 10);
  }
  require(load.level() == 0);
  // Severe work may complete only one or two LIO cycles per second. A
  // throughput floor must not disable adaptation exactly in this condition.
  for (const std::size_t cycles : {0, 1, 2}) {
    AdaptiveLoad slow; slow.enable(true);
    for (int window = 0; window < 10; ++window) slow.observe(1., .99, 2., cycles);
    require(slow.level() == (cycles == 0 ? 0u : 7u));
  }
  for (unsigned target = 1; target <= 7; ++target) {
    load.observe(1., .95, .3, 10); require(load.level() == target - 1);
    load.observe(1., .95, .3, 10); require(load.level() == target);
    require(load.vioInterval(.2) >= .2 && load.vioInterval(.2) <= .5);
    constexpr double expected[] = {1., 1., 1., 1., .75, .5, 1. / 3., .25};
    require(load.queryFraction() == expected[target]);
  }
  require(load.vioInterval(.2) == .5 && load.queryFraction() == .25);
  require(load.vioInterval(.8) == .8); // Explicit slower configuration wins.
  for (int i = 0; i < 30; ++i) load.observe(1., .99, 1., 10);
  require(load.level() == 7);
  for (unsigned target = 7; target > 0; --target) {
    for (int i = 0; i < 7; ++i) load.observe(1., .3, .02, 10);
    require(load.level() == target);
    load.observe(1., .3, .02, 10); require(load.level() == target - 1);
  }
  require(load.changes() == 21 && load.vioInterval(.2) == .2);
  // Severe pressure can act after one complete window, but only one level.
  load.observe(20., 19., 2., 100); require(load.level() == 1);
  load.enable(false); load.enable(true);
  load.observe(.9, .89, 2., 1); require(load.level() == 0);
  load.observe(1., .95, .8, 1); require(load.level() == 1);
  load.enable(false); load.enable(true);
  load.observe(1., std::numeric_limits<double>::quiet_NaN(), 2., 10);
  load.observe(1., .95, .3, 10); require(load.level() == 0);
  load.observe(1., .95, .3, 10); require(load.level() == 1);
  load.enable(false); require(load.level() == 0 && load.queryFraction() == 1.);

  for (const std::size_t size : {0, 1, 32, 999, 1000, 1001, 4097, 12001}) {
    for (double fraction : {1., .75, .5, 1. / 3., .25}) {
      const auto selected = matchQueryCount(size, fraction);
      require(selected <= size && (size < 1000 || selected >= 1000));
      std::set<std::size_t> indices;
      for (std::size_t i = 0; i < selected; ++i) {
        const auto index = matchQueryIndex(i, selected, size);
        require(index < size && indices.insert(index).second);
        if (selected == size) require(index == i);
      }
      // No long run of spatially sorted points is omitted, even at 1/4 load.
      if (!indices.empty()) {
        const auto spacing = (size + selected - 1) / selected;
        require(spacing <= 4);
        require(*indices.begin() < spacing && size - 1 - *indices.rbegin() < spacing);
        auto previous = *indices.begin();
        for (const auto index : indices) { require(index - previous <= spacing); previous = index; }
      }
    }
  }
  for (const std::size_t size : {3000, 9000, 18000, 36000}) {
    require(matchQueryCount(size, 1., 4000) == std::min<std::size_t>(size, 4000));
    require(matchQueryCount(size, .25, 4000) == 1000);
    const auto selected = matchQueryCount(size, 1., 4000);
    std::size_t previous = 0;
    for (std::size_t i = 0; i < selected; ++i) {
      const auto index = matchQueryIndex(i, selected, size);
      require(index < size && (i == 0 || index > previous));
      previous = index;
    }
  }
  std::cout << "Adaptive pressure, hysteresis, delayed-input exclusion and bounded query coverage passed\n";
}
