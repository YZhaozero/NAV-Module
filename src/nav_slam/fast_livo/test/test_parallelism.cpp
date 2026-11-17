#include "parallelism.h"
#include <stdexcept>
#include <thread>
#include <vector>

static void require(bool value) {
  if (!value) throw std::runtime_error("OpenMP deployment budget regression");
}

int main() {
  using fast_livo_parallel::configuredThreads;
  require(configuredThreads(nullptr, 4, 6) == 4);
  require(configuredThreads("2", 4, 6) == 2);
  require(configuredThreads("1", 4, 6) == 1);
  require(configuredThreads("99", 4, 6) == 4);
  require(configuredThreads("4", 4, 2) == 2);
  require(configuredThreads("2", 4, 1) == 1);
  require(configuredThreads("4", 4, 0) == 4);
  require(configuredThreads(" 2, 3 ", 4, 6) == 2);
  require(configuredThreads(" +2 ", 4, 6) == 2);
  for (const char *bad : {"", " ", "0", "-1", "x", "2x", "2,", "2,0",
                          "99999999999999999999999999999999999999999"})
    require(configuredThreads(bad, 4, 6) == 4);
  require(configuredThreads("2", 1, 6) == 1);
  const int expected = fast_livo_parallel::threadCount();
  require(expected >= 1 && expected <= 4);
  std::vector<int> observed(8, 0);
  std::vector<std::thread> threads;
  for (std::size_t i = 0; i < observed.size(); ++i)
    threads.emplace_back([&, i] { observed[i] = fast_livo_parallel::threadCount(); });
  for (auto &thread : threads) thread.join();
  for (int value : observed) require(value == expected);
#ifdef __linux__
  cpu_set_t original;
  CPU_ZERO(&original);
  require(sched_getaffinity(0, sizeof(original), &original) == 0);
  struct RestoreAffinity {
    cpu_set_t mask;
    ~RestoreAffinity() { sched_setaffinity(0, sizeof(mask), &mask); }
  } restore{original};
  cpu_set_t single;
  CPU_ZERO(&single);
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &original)) { CPU_SET(cpu, &single); break; }
  }
  require(sched_setaffinity(0, sizeof(single), &single) == 0);
#ifdef _OPENMP
  if (omp_get_proc_bind() != omp_proc_bind_false)
    require(fast_livo_parallel::threadCount() == expected);
  else
#endif
  require(fast_livo_parallel::threadCount() == 1);
  require(sched_setaffinity(0, sizeof(original), &original) == 0);
  require(fast_livo_parallel::threadCount() == expected);
#endif
}
