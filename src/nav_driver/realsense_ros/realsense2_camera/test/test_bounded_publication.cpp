#include "bounded_publication.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <future>
#include <stdexcept>
#include <vector>

using namespace realsense2_camera;
using namespace std::chrono;
using Item = std::unique_ptr<int>;
using Worker = BoundedPublication<Item>;
Item item(int id) { return Item(new int(id)); }
template<class Predicate> void wait_until(Predicate predicate) {
    const auto deadline = steady_clock::now()+seconds(2);
    while (!predicate()) {
        assert(steady_clock::now() < deadline);
        std::this_thread::sleep_for(milliseconds(1));
    }
}

void stalled_sink_does_not_block_producer_and_keeps_latest_bounded() {
    std::promise<void> entered, unblock;
    auto released = unblock.get_future().share();
    std::mutex mutex;
    std::vector<int> received;
    Worker worker(3, 30, [&](Item&& value) {
        if (*value == 1) { entered.set_value(); released.wait(); }
        std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);
    });
    assert(worker.submit(item(1), 10)); entered.get_future().wait();
    auto producer = std::async(std::launch::async, [&] {
        assert(worker.submit(item(2), 10));
        assert(worker.submit(item(3), 10));
        assert(worker.submit(item(4), 10));
        assert(!worker.submit(item(5), 31));  // Oversize never evicts existing data.
        assert(!worker.submit(item(6), 25));  // In-flight reservation remains counted.
    });
    assert(producer.wait_for(milliseconds(250)) == std::future_status::ready);
    producer.get();
    const auto blocked = worker.stats();
    assert(blocked.resident_items == 3 && blocked.resident_bytes == 30);
    assert(blocked.peak_bytes == 30 && blocked.dropped_oldest == 1);
    assert(blocked.rejected_oversize == 1 && blocked.rejected_busy == 1);
    unblock.set_value();
    wait_until([&] { return worker.stats().published == 3; });
    worker.stop();
    assert((received == std::vector<int>{1, 3, 4}));
    assert(worker.stats().resident_bytes == 0);
}

void normal_frames_keep_order_and_move_ownership() {
    std::vector<int> received;
    Worker worker(3, 64, [&](Item&& value) { received.push_back(*value); });
    for (int n=0; n<50; ++n) {
        auto frame = item(n);
        assert(worker.submit(std::move(frame), 10)); assert(!frame);
        wait_until([&] { return worker.stats().published == unsigned(n+1); });
    }
    worker.stop();
    assert(received.size() == 50);
    for (int n=0; n<50; ++n) assert(received[n] == n);
    assert(worker.stats().dropped_oldest == 0);
}

void stop_rejects_new_frames_discards_pending_and_joins_inflight() {
    std::promise<void> entered, unblock;
    auto released = unblock.get_future().share();
    std::vector<int> received;
    Worker worker(3, 30, [&](Item&& value) {
        entered.set_value(); released.wait(); received.push_back(*value);
    });
    assert(worker.submit(item(1), 10)); entered.get_future().wait();
    assert(worker.submit(item(2), 10)); assert(worker.submit(item(3), 10));
    worker.requestStop();
    assert(!worker.submit(item(4), 10));
    assert(worker.stats().discarded_on_stop == 2);
    assert(worker.stats().resident_bytes == 10);
    auto joining = std::async(std::launch::async, [&] { worker.join(); });
    assert(joining.wait_for(milliseconds(25)) == std::future_status::timeout);
    unblock.set_value(); joining.get();
    assert((received == std::vector<int>{1}));
    assert(worker.stats().resident_bytes == 0);
}

void sink_failure_does_not_kill_worker() {
    Worker worker(3, 30, [](Item&& value) { if (*value == 1) throw std::runtime_error("test sink"); });
    assert(worker.submit(item(1), 10)); assert(worker.submit(item(2), 10));
    wait_until([&] { const auto s=worker.stats(); return s.failures == 1 && s.published == 1; });
    worker.stop(); assert(worker.stats().resident_items == 0);
}

void separate_streams_do_not_share_sink_backpressure() {
    std::promise<void> entered, unblock;
    auto released = unblock.get_future().share();
    Worker depth(3, 30, [&](Item&&) { entered.set_value(); released.wait(); });
    Worker rgb(3, 30, [](Item&&) {});
    assert(depth.submit(item(1), 10)); entered.get_future().wait();
    for (int n=0; n<20; ++n) {
        assert(rgb.submit(item(n), 10));
        wait_until([&] { return rgb.stats().published == unsigned(n+1); });
    }
    assert(depth.stats().published == 0);
    unblock.set_value(); depth.stop(); rgb.stop();
}

void closed_profile_gate_rejects_and_waits_only_for_existing_callbacks() {
    PublicationCallbackGate gate;
    assert(!gate.enter()); gate.open();
    auto lease = std::unique_ptr<PublicationCallbackGate::Lease>(new PublicationCallbackGate::Lease(gate.enter()));
    assert(bool(*lease));
    auto closed = std::async(std::launch::async, [&] { gate.closeAndWait(); });
    wait_until([&] { return !gate.enter(); });
    assert(closed.wait_for(milliseconds(25)) == std::future_status::timeout);
    lease.reset(); closed.get();
    assert(!gate.enter()); gate.open(); assert(bool(gate.enter()));
    gate.closeAndWait();
}

void failed_profile_reconfiguration_reopens_ingress() {
    PublicationCallbackGate gate;
    gate.open();
    bool partial_publisher = false;
    try {
        PublicationCallbackGate::Reconfiguration reconfigure(gate);
        assert(!gate.enter());
        try {
            partial_publisher = true;
            throw std::runtime_error("sensor start failure");
        } catch (...) {
            partial_publisher = false;
            throw;
        }
    } catch (const std::runtime_error&) {}
    assert(!partial_publisher);
    assert(bool(gate.enter()));
    {
        PublicationCallbackGate::Reconfiguration reconfigure(gate);
        assert(!gate.enter());
    }
    assert(bool(gate.enter()));
    gate.closeAndWait();
}

int main() {
    stalled_sink_does_not_block_producer_and_keeps_latest_bounded();
    normal_frames_keep_order_and_move_ownership();
    stop_rejects_new_frames_discards_pending_and_joins_inflight();
    sink_failure_does_not_kill_worker();
    separate_streams_do_not_share_sink_backpressure();
    closed_profile_gate_rejects_and_waits_only_for_existing_callbacks();
    failed_profile_reconfiguration_reopens_ingress();
}
