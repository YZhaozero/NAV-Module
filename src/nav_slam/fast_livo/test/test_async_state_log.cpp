#include "async_state_log.h"
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <filesystem>
#include <unistd.h>

static void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  std::promise<void> entered, release;
  auto blocked = release.get_future().share();
  std::vector<std::pair<unsigned, std::string>> written;
  {
    AsyncStateLog log([&](unsigned channel, const std::string &line) {
      if (written.empty()) { entered.set_value(); blocked.wait(); }
      written.push_back({channel, line});
    }, 16);
    require(log.append(0, "first"), "first row rejected");
    entered.get_future().wait();
    // Keep the disk blocked while the estimator fills its bounded queue.
    auto producer = std::async(std::launch::async, [&] {
      require(log.append(1, "second"), "second row rejected");
      require(log.append(0, "third"), "third row rejected");
      require(!log.append(1, "over capacity"), "unbounded queue");
    });
    const bool nonblocking = producer.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready;
    release.set_value();
    producer.get();
    require(nonblocking, "slow disk blocked estimator");
    require(log.dropped() == 1, "missing dropped-row diagnostic");
  }
  require(written.size() == 3, "shutdown did not drain pending rows");
  require(written[0] == std::make_pair(0u, std::string("first")) &&
          written[1] == std::make_pair(1u, std::string("second")) &&
          written[2] == std::make_pair(0u, std::string("third")), "row order/channel changed");
  {
    std::promise<void> attempted;
    AsyncStateLog log([&](unsigned, const std::string &) {
      attempted.set_value(); throw std::runtime_error("disk failure");
    });
    log.append(0, "row");
    attempted.get_future().wait();
    for (unsigned i=0; i<1000 && !log.errors(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    require(log.errors() == 1, "disk error not reported");
  }
  {
    const auto folder = std::filesystem::temp_directory_path() /
        ("nav-async-imu-" + std::to_string(getpid()) + "-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto path = folder / "nested" / "imu.txt";
    {
      AsyncStateLog log(std::array<std::string,2>{path.string(), ""});
      for (unsigned i=0; i<200; ++i)
        require(log.append(0, std::to_string(i)+" imu\n"), "IMU row rejected");
      require(log.errors() == 0, "unused file channel counted as an error");
    }
    {
      AsyncStateLog reopened(std::array<std::string,2>{path.string(), ""});
      require(reopened.append(0, "after reopen\n"), "reopened logger rejected data");
    }
    std::ifstream input(path);
    std::string row;
    for (unsigned i=0; i<200; ++i) {
      require(bool(std::getline(input,row)), "IMU shutdown did not flush all rows");
      require(row==std::to_string(i)+" imu", "IMU row changed or reordered");
    }
    require(bool(std::getline(input,row)) && row == "after reopen", "reopen truncated earlier evidence");
    require(!std::getline(input,row), "unexpected IMU row");
    input.close();
    const auto blocked = folder / "not-a-directory";
    { std::ofstream file(blocked); file << "preserve"; }
    {
      AsyncStateLog failed(std::array<std::string,2>{(blocked / "imu.txt").string(), ""});
      for (unsigned i=0; i<1000 && !failed.errors(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      require(failed.errors() > 0, "directory creation failure was not reported");
    }
    std::filesystem::remove_all(folder);
  }
  std::cout << "async state log: slow disk, bounded queue, draining and errors passed\n";
}
