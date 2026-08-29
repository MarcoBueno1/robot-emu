// src/safety/watchdog.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/safety/watchdog.hpp"

namespace robot::safety {

namespace {
[[nodiscard]] std::int64_t nowNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

Watchdog::Watchdog(HeartbeatCounterFn heartbeatCounter, std::chrono::nanoseconds timeout,
                    std::chrono::nanoseconds pollInterval) noexcept
    : heartbeatCounter_(std::move(heartbeatCounter)), timeout_(timeout), pollInterval_(pollInterval) {}

Watchdog::~Watchdog() {
    if (running_.load(std::memory_order_acquire)) {
        running_.store(false, std::memory_order_release);
        thread_.request_stop();
        // thread_'s own destructor joins automatically — see
        // robot::runtime::ControlLoop's destructor (Phase 3) for the same
        // idiom and its documented bounded-latency caveat.
    }
}

std::expected<void, WatchdogError> Watchdog::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return std::unexpected(WatchdogError::AlreadyRunning);
    }

    lastSeenCounter_.store(heartbeatCounter_(), std::memory_order_relaxed);
    lastChangeTimeNs_.store(nowNs(), std::memory_order_relaxed);
    tripped_.store(false, std::memory_order_relaxed);

    thread_ = std::jthread([this](std::stop_token stopToken) { runLoop(stopToken); });
    return {};
}

std::expected<void, WatchdogError> Watchdog::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        return std::unexpected(WatchdogError::NotRunning);
    }
    thread_.request_stop();
    if (thread_.joinable()) {
        thread_.join();
    }
    return {};
}

bool Watchdog::isRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
}

bool Watchdog::tripped() const noexcept {
    return tripped_.load(std::memory_order_acquire);
}

void Watchdog::reset() noexcept {
    tripped_.store(false, std::memory_order_release);
    lastSeenCounter_.store(heartbeatCounter_(), std::memory_order_relaxed);
    lastChangeTimeNs_.store(nowNs(), std::memory_order_relaxed);
}

void Watchdog::runLoop(std::stop_token stopToken) {
    while (!stopToken.stop_requested()) {
        std::this_thread::sleep_for(pollInterval_);
        if (stopToken.stop_requested()) {
            break;
        }

        const std::uint64_t current = heartbeatCounter_();
        const std::uint64_t previous = lastSeenCounter_.load(std::memory_order_relaxed);
        const std::int64_t now = nowNs();

        if (current != previous) {
            lastSeenCounter_.store(current, std::memory_order_relaxed);
            lastChangeTimeNs_.store(now, std::memory_order_relaxed);
        } else {
            const std::int64_t lastChange = lastChangeTimeNs_.load(std::memory_order_relaxed);
            if (now - lastChange >= timeout_.count()) {
                tripped_.store(true, std::memory_order_release);
            }
        }
    }
}

}  // namespace robot::safety
