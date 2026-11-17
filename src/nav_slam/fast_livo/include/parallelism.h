#pragma once

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <thread>
#ifdef _OPENMP
#include <omp.h>
#endif
#ifdef __linux__
#include <sched.h>
#endif

namespace fast_livo_parallel {

// Honor the outer team size of OpenMP's comma-separated nested-team syntax.
// Invalid values use the existing compiled budget; neither runtime requests nor
// a larger host can exceed the estimator's tested compile-time upper bound.
inline int configuredThreads(const char *setting, int compiled_limit,
                             unsigned hardware_threads) {
  const int hardware_limit = hardware_threads ?
      static_cast<int>(std::min<unsigned>(hardware_threads, 1024)) : compiled_limit;
  const int limit = std::max(1, std::min(compiled_limit, hardware_limit));
  if (!setting) return limit;
  const char *cursor = setting;
  long first = 0;
  bool have_first = false;
  for (;;) {
    while (std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (!*cursor) return limit;
    char *end = nullptr;
    errno = 0;
    const long value = std::strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE || value < 1) return limit;
    if (!have_first) { first = value; have_first = true; }
    cursor = end;
    while (std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    if (!*cursor) return static_cast<int>(std::min<long>(first, limit));
    if (*cursor++ != ',') return limit;
  }
}

inline int threadCount() {
#ifdef MP_PROC_NUM
  constexpr int compiled_limit = MP_PROC_NUM;
#else
  constexpr int compiled_limit = 4;
#endif
  // Capture the deployment budget once, before any OpenMP region. Do not use
  // omp_set_num_threads: it overwrites the inherited budget for later regions.
  static const int count = configuredThreads(std::getenv("OMP_NUM_THREADS"),
      compiled_limit, std::thread::hardware_concurrency());
#ifdef _OPENMP
  // OpenMP binding may intentionally pin just the primary thread to one
  // place, while the rest of its team uses other places. Preserve that policy.
  if (omp_get_proc_bind() != omp_proc_bind_false) return count;
#endif
  // A service/cpuset can change the available CPUs while the estimator runs.
  // Respect the calling estimator thread's current affinity instead of
  // oversubscribing that smaller set with the original OpenMP team.
#ifdef __linux__
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  if (sched_getaffinity(0, sizeof(allowed), &allowed) == 0) {
    const int available = CPU_COUNT(&allowed);
    if (available > 0) return std::min(count, available);
  }
#endif
  return count;
}

}  // namespace fast_livo_parallel
