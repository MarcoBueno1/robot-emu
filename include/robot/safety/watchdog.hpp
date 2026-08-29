// include/robot/safety/watchdog.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <thread>
#include "robot/safety/watchdog_error.hpp"

namespace robot::safety {

/// @brief An independently-threaded liveness monitor.
///
/// Fed a heartbeat counter expected to keep increasing, Watchdog trips if
/// that counter stops advancing for longer than a configured timeout —
/// this is docs/architecture.md section 3.15's "safety watchdog
/// independent from the main control loop... running on a separate
/// thread, so a control-loop lockup doesn't prevent fault detection."
///
/// Watchdog has no dependency on robot_runtime — any HeartbeatCounterFn
/// works, including but not limited to robot::runtime::ControlLoop's
/// cyclesExecuted metric. start()/stop()/isRunning()'s shape deliberately
/// mirrors ControlLoop's (Phase 3) exactly.
class Watchdog {
public:
    using HeartbeatCounterFn = std::function<std::uint64_t()>;

    /// @param heartbeatCounter Returns a counter expected to keep
    ///        increasing. Must be safe to call from this watchdog's own
    ///        background thread, concurrently with whatever thread
    ///        updates the underlying counter.
    /// @param timeout If heartbeatCounter() hasn't advanced for this long, tripped() becomes true.
    /// @param pollInterval How often the background thread checks.
    Watchdog(HeartbeatCounterFn heartbeatCounter, std::chrono::nanoseconds timeout,
             std::chrono::nanoseconds pollInterval) noexcept;

    /// @brief Stops the background thread if still running.
    ~Watchdog();

    Watchdog(const Watchdog&) = delete;
    Watchdog& operator=(const Watchdog&) = delete;
    Watchdog(Watchdog&&) = delete;
    Watchdog& operator=(Watchdog&&) = delete;

    /// @brief Starts the background monitoring thread.
    /// @return Success, or WatchdogError::AlreadyRunning.
    [[nodiscard]] std::expected<void, WatchdogError> start();

    /// @brief Signals the background thread to stop and blocks until it exits.
    /// @return Success, or WatchdogError::NotRunning.
    [[nodiscard]] std::expected<void, WatchdogError> stop();

    /// @brief Whether a background thread started by start() is currently active.
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief True once the heartbeat counter has been observed to not
    ///        advance for at least timeout. Sticky — stays true until reset().
    [[nodiscard]] bool tripped() const noexcept;

    /// @brief Clears tripped() and re-baselines the internal "last seen
    ///        counter/time" so the watchdog doesn't immediately re-trip on
    ///        its next poll.
    void reset() noexcept;

private:
    void runLoop(std::stop_token stopToken);

    HeartbeatCounterFn heartbeatCounter_;
    std::chrono::nanoseconds timeout_;
    std::chrono::nanoseconds pollInterval_;

    std::atomic<bool> running_{false};
    std::atomic<bool> tripped_{false};
    std::jthread thread_;

    std::atomic<std::uint64_t> lastSeenCounter_{0};
    std::atomic<std::int64_t> lastChangeTimeNs_{0};
};

}  // namespace robot::safety
