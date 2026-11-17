// Copyright 2026. Licensed under the Apache License, Version 2.0.
#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace realsense2_camera {

// Owns messages, never sensor buffers. The budget includes the item currently
// inside the sink. The sink is never called while the queue mutex is held.
template<class T> class BoundedPublication {
public:
    struct Stats {
        uint64_t accepted = 0, published = 0, failures = 0;
        uint64_t dropped_oldest = 0, rejected_oversize = 0;
        uint64_t rejected_busy = 0, rejected_stopped = 0, discarded_on_stop = 0;
        size_t resident_items = 0, resident_bytes = 0, peak_bytes = 0;
    };
    using Sink = std::function<void(T&&)>;
    BoundedPublication(size_t max_items, size_t max_bytes, Sink sink)
      : _max_items(max_items), _max_bytes(max_bytes), _sink(std::move(sink)),
        _thread([this] { run(); }) {}
    ~BoundedPublication() { stop(); }
    BoundedPublication(const BoundedPublication&) = delete;
    BoundedPublication& operator=(const BoundedPublication&) = delete;

    bool submit(T item, size_t bytes) {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_accepting) { ++_stats.rejected_stopped; return false; }
        if (!_max_items || bytes > _max_bytes) { ++_stats.rejected_oversize; return false; }
        if ((_active && _max_items == 1) || bytes > _max_bytes-_active_bytes) {
            ++_stats.rejected_busy; return false;
        }
        while (!_pending.empty() && (_stats.resident_items >= _max_items ||
                                     bytes > _max_bytes-_stats.resident_bytes)) {
            removeFront(); ++_stats.dropped_oldest;
        }
        if (_stats.resident_items >= _max_items || bytes > _max_bytes-_stats.resident_bytes) {
            ++_stats.rejected_busy; return false;
        }
        _pending.push_back(Entry{std::move(item), bytes});
        ++_stats.accepted; ++_stats.resident_items; _stats.resident_bytes += bytes;
        if (_stats.resident_bytes > _stats.peak_bytes) _stats.peak_bytes = _stats.resident_bytes;
        _cv.notify_one();
        return true;
    }
    void requestStop() {
        std::lock_guard<std::mutex> lock(_mutex);
        _accepting = false;
        while (!_pending.empty()) { removeFront(); ++_stats.discarded_on_stop; }
        _cv.notify_all();
    }
    void join() { if (_thread.joinable()) _thread.join(); }
    void stop() { requestStop(); join(); }
    Stats stats() const { std::lock_guard<std::mutex> lock(_mutex); return _stats; }
private:
    struct Entry { T item; size_t bytes; };
    void removeFront() {
        _stats.resident_bytes -= _pending.front().bytes;
        --_stats.resident_items; _pending.pop_front();
    }
    void run() {
        for (;;) {
            std::unique_ptr<Entry> current;
            {
                std::unique_lock<std::mutex> lock(_mutex);
                _cv.wait(lock, [this] { return !_accepting || !_pending.empty(); });
                if (!_accepting) return;
                current.reset(new Entry(std::move(_pending.front())));
                _pending.pop_front();  // Still resident until the sink returns.
                _active = true; _active_bytes = current->bytes;
            }
            bool success = false;
            try { _sink(std::move(current->item)); success = true; } catch (...) {}
            const size_t bytes = current->bytes;
            current.reset();
            {
                std::lock_guard<std::mutex> lock(_mutex);
                if (success) ++_stats.published; else ++_stats.failures;
                _stats.resident_bytes -= bytes;
                --_stats.resident_items;
                _active = false; _active_bytes = 0;
            }
        }
    }
    const size_t _max_items, _max_bytes;
    Sink _sink;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::deque<Entry> _pending;
    bool _accepting = true;
    bool _active = false;
    size_t _active_bytes = 0;
    Stats _stats;
    std::thread _thread;
};

// Profile changes close ingress, wait for only the short SDK preparation work,
// then stop workers before replacing publishers. A closed gate rejects rather
// than making the SDK callback wait behind profile reconfiguration.
class PublicationCallbackGate {
public:
    // Profile reconfiguration must release ingress on exception as well as on
    // success. The caller first removes any partially initialized publishers.
    // Permanent shutdown deliberately uses closeAndWait() instead of this scope.
    class Reconfiguration {
    public:
        explicit Reconfiguration(PublicationCallbackGate& gate) : _gate(gate) {
            _gate.closeAndWait();
        }
        ~Reconfiguration() { _gate.open(); }
        Reconfiguration(const Reconfiguration&) = delete;
        Reconfiguration& operator=(const Reconfiguration&) = delete;
    private:
        PublicationCallbackGate& _gate;
    };
    class Lease {
    public:
        explicit Lease(PublicationCallbackGate * gate = nullptr) : _gate(gate) {}
        Lease(Lease&& other) noexcept : _gate(other._gate) { other._gate = nullptr; }
        Lease(const Lease&) = delete;
        Lease& operator=(const Lease&) = delete;
        ~Lease() { if (_gate) _gate->leave(); }
        explicit operator bool() const { return _gate != nullptr; }
    private:
        PublicationCallbackGate * _gate;
    };
    Lease enter() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_open) return Lease();
        ++_active; return Lease(this);
    }
    void open() { std::lock_guard<std::mutex> lock(_mutex); _open = true; }
    void closeAndWait() {
        std::unique_lock<std::mutex> lock(_mutex);
        _open = false; _cv.wait(lock, [this] { return _active == 0; });
    }
private:
    void leave() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (--_active == 0) _cv.notify_all();
    }
    std::mutex _mutex;
    std::condition_variable _cv;
    bool _open = false;
    size_t _active = 0;
};
}  // namespace realsense2_camera
