#include "latest_output_worker.h"

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;
static void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}

struct Gate {
  std::mutex mutex;
  std::condition_variable cv;
  bool entered = false, released = false;
  void block() {
    std::unique_lock<std::mutex> lock(mutex);
    entered = true;
    cv.notify_all();
    cv.wait(lock, [&] { return released; });
  }
  bool awaitEntry() {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, 5s, [&] { return entered; });
  }
  void release() {
    { std::lock_guard<std::mutex> lock(mutex); released = true; }
    cv.notify_all();
  }
};

struct Payload {
  int id;
  std::atomic<int> &live, &peak;
  Payload(int value, std::atomic<int> &count, std::atomic<int> &maximum)
      : id(value), live(count), peak(maximum) {
    const int current = ++live;
    int before = peak.load();
    while (before < current && !peak.compare_exchange_weak(before, current)) {}
  }
  ~Payload() { --live; }
};

static void newestWinsWithoutWaiting() {
  Gate gate;
  std::atomic<int> live{0}, peak{0};
  std::vector<int> delivered;
  fast_livo_output::LatestOutputWorker<Payload> worker([&](auto value) {
    if (value->id == 0) gate.block();
    delivered.push_back(value->id);
  });
  worker.submit(std::make_shared<Payload>(0, live, peak));
  if (!gate.awaitEntry()) { gate.release(); throw std::runtime_error("Sink did not start"); }
  auto producer = std::async(std::launch::async, [&] {
    for (int i = 1; i <= 10000; ++i)
      require(worker.submit(std::make_shared<Payload>(i, live, peak)), "Unexpected rejection");
  });
  const bool submitted_before_release = producer.wait_for(5s) == std::future_status::ready;
  const auto blocked = worker.stats();
  gate.release();
  producer.get();
  worker.stop();
  require(submitted_before_release, "Submitting waited for the active callback");
  require(blocked.active && blocked.pending && blocked.submitted == 10001 &&
          blocked.replaced == 9999, "Queue retained more than the newest pending item");
  require(delivered == std::vector<int>({0, 10000}), "Stale display jobs were executed");
  // The producer temporarily owns its newly allocated argument in addition to
  // the worker's active and pending payloads.
  require(peak <= 3 && live == 0, "Payload lifetime was unbounded or leaked");
  const auto stopped = worker.stats();
  require(stopped.completed == 2 && !stopped.active && !stopped.pending && !stopped.accepting,
          "Draining shutdown did not finish the newest job");
}

static void discardAndConcurrentStop() {
  Gate gate;
  std::atomic<int> callbacks{0};
  fast_livo_output::LatestOutputWorker<int> worker([&](auto) { ++callbacks; gate.block(); });
  worker.submit(std::make_shared<const int>(0));
  if (!gate.awaitEntry()) { gate.release(); throw std::runtime_error("Sink did not start"); }
  worker.submit(std::make_shared<const int>(1));
  auto first = std::async(std::launch::async, [&] { worker.stop(false); });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (worker.stats().accepting && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();
  const bool closed = !worker.stats().accepting;
  auto second = std::async(std::launch::async, [&] { worker.stop(); });
  auto producer = std::async(std::launch::async, [&] {
    for (int i = 0; i < 10000; ++i)
      require(!worker.submit(std::make_shared<const int>(i)), "Accepted after shutdown");
  });
  const bool rejected_before_release = producer.wait_for(5s) == std::future_status::ready;
  gate.release();
  producer.get(); first.get(); second.get();
  require(closed && rejected_before_release && callbacks == 1 && worker.stats().discarded == 1,
          "Concurrent stopping blocked submit or ran a discarded job");
}

static void releaseOutsideQueueLock() {
  Gate gate;
  std::atomic<int> deleted{0};
  fast_livo_output::LatestOutputWorker<int> worker([&](auto) { gate.block(); });
  worker.submit(std::make_shared<const int>(0));
  if (!gate.awaitEntry()) { gate.release(); throw std::runtime_error("Sink did not start"); }
  worker.submit(std::shared_ptr<const int>(new int(1), [&](const int *p) {
    (void)worker.stats();  // Would deadlock if replaced storage were released under the mutex.
    ++deleted; delete p;
  }));
  worker.submit(std::make_shared<const int>(2));
  require(deleted == 1, "Replaced item remained retained");
  gate.release(); worker.stop();
}

static void producersRaceWithStop() {
  Gate gate;
  std::atomic<int> attempted{0}, accepted{0}, callbacks{0};
  fast_livo_output::LatestOutputWorker<int> worker([&](auto) { ++callbacks; gate.block(); });
  worker.submit(std::make_shared<const int>(0));
  if (!gate.awaitEntry()) { gate.release(); throw std::runtime_error("Sink did not start"); }
  std::vector<std::future<void>> producers;
  for (int producer = 0; producer < 4; ++producer) {
    producers.push_back(std::async(std::launch::async, [&] {
      for (int i = 0; i < 10000; ++i) {
        if (worker.submit(std::make_shared<const int>(i))) ++accepted;
        ++attempted;
      }
    }));
  }
  auto stopper = std::async(std::launch::async, [&] {
    while (attempted < 32) std::this_thread::yield();
    worker.stop(false);
  });
  bool ready = true;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  for (auto &producer : producers)
    ready = (producer.wait_until(deadline) == std::future_status::ready) && ready;
  gate.release();
  for (auto &producer : producers) producer.get();
  stopper.get();
  const auto stopped = worker.stats();
  require(ready && attempted == 40000 && callbacks == 1 && !stopped.pending &&
          stopped.submitted == static_cast<std::uint64_t>(accepted + 1) &&
          stopped.replaced + stopped.discarded == static_cast<std::uint64_t>(accepted),
          "Concurrent producer/shutdown accounting or progress was incorrect");
}

static void failureIsVisibleAndDoesNotTerminate() {
  Gate gate;
  std::atomic<int> live{0}, peak{0};
  fast_livo_output::LatestOutputWorker<Payload> worker([&](auto) {
    gate.block(); throw std::runtime_error("sink failed");
  });
  worker.submit(std::make_shared<Payload>(0, live, peak));
  if (!gate.awaitEntry()) { gate.release(); throw std::runtime_error("Sink did not start"); }
  worker.submit(std::make_shared<Payload>(1, live, peak));
  gate.release(); worker.stop();
  auto throws_failure = [](auto action) {
    try { action(); }
    catch (const std::runtime_error &error) { return std::string(error.what()) == "sink failed"; }
    return false;
  };
  require(throws_failure([&] { worker.rethrowIfFailed(); }), "Sink failure was hidden");
  require(throws_failure([&] { worker.submit(std::make_shared<Payload>(2, live, peak)); }),
          "Submit did not propagate sink failure");
  const auto failed = worker.stats();
  require(failed.failed && !failed.accepting && !failed.pending && !failed.active &&
          failed.discarded == 1 && live == 0, "Failure left payloads alive");
  worker.stop();  // Repeated shutdown must not rethrow the background exception.
}

static void callbackCanRequestStop() {
  fast_livo_output::LatestOutputWorker<int> *target = nullptr;
  fast_livo_output::LatestOutputWorker<int> worker([&](auto) { target->stop(false); });
  target = &worker;
  worker.submit(std::make_shared<const int>(1));
  worker.stop();
  require(worker.stats().completed == 1, "Self-stop deadlocked or lost the active callback");
}

static void callbackCanReleaseItsOwner() {
  using Worker = fast_livo_output::LatestOutputWorker<int>;
  std::promise<void> submitted;
  auto may_destroy = submitted.get_future().share();
  auto released = std::make_shared<std::promise<void>>();
  auto completion = released->get_future();
  std::atomic<bool> callback_finished{false};
  std::unique_ptr<Worker> owner;
  owner = std::make_unique<Worker>([&](auto) {
    may_destroy.wait();  // Destruction must not race the initial submit call.
    owner.reset();
    callback_finished = true;
  });
  // This deleter runs after the callback has returned, when work() releases
  // its active item. It also owns the promise until set_value() has returned.
  owner->submit(std::shared_ptr<const int>(new int(1), [released](const int *p) {
    delete p;
    released->set_value();
  }));
  submitted.set_value();
  require(completion.wait_for(5s) == std::future_status::ready && callback_finished,
          "Destroying the owner inside its callback invalidated worker state");
  require(!owner, "Callback did not release its owner");
}

int main() {
  newestWinsWithoutWaiting();
  discardAndConcurrentStop();
  releaseOutsideQueueLock();
  producersRaceWithStop();
  failureIsVisibleAndDoesNotTerminate();
  callbackCanRequestStop();
  callbackCanReleaseItsOwner();
  std::cout << "Latest output worker concurrency and lifetime tests passed\n";
}
