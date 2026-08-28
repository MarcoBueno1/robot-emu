// include/robot/runtime/control_loop.hpp
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
#include <expected>
#include <thread>
#include "robot/controller/controller_state_machine.hpp"
#include "robot/core/robot.hpp"
#include "robot/runtime/control_loop_error.hpp"
#include "robot/runtime/control_loop_frequency.hpp"
#include "robot/runtime/control_loop_metrics.hpp"

namespace robot::runtime {

/// @brief Ties a Robot and a ControllerStateMachine together and advances
///        them at a configurable fixed frequency.
///
/// ControlLoop is the first type in this codebase that spans both
/// robot::core and robot::controller, and the first to use real threading.
/// It borrows both dependencies (does not own them) — the referenced Robot
/// and ControllerStateMachine must outlive this ControlLoop, including for
/// the lifetime of any running background thread.
///
/// Two ways to advance time:
///  - step(): one cycle, synchronously, on the caller's thread. No sleep,
///    no thread, fully deterministic — the unit for tests.
///  - start()/stop(): a background std::jthread repeatedly calls step(),
///    paced to period() via std::this_thread::sleep_until against
///    std::chrono::steady_clock. See docs/task-briefs/phase-03-control-loop.md
///    section 6 ("Why no injected Clock") for why this path does not use
///    robot::core::Clock.
///
/// Known bounded latency: stop() / the destructor may block for up to
/// approximately one period() before the background thread notices a
/// stop request and exits, because std::this_thread::sleep_until is not
/// itself interrupted by a stop_token — the loop only checks
/// stop_requested() once per cycle. Acceptable for this phase's soft
/// real-time scope (periods here are at most 2 ms, at the slowest
/// configured frequency).
class ControlLoop {
public:
    /// @brief Constructs a ControlLoop bound to robot and controller.
    /// @param robot Robot to advance. Must outlive this ControlLoop.
    /// @param controller State machine whose state() gates motion. Must
    ///        outlive this ControlLoop. Never mutated by ControlLoop —
    ///        only state() is read; handleEvent() is never called here.
    /// @param frequency Fixed cycle frequency; see period().
    ControlLoop(robot::core::Robot& robot,
                robot::controller::ControllerStateMachine& controller,
                ControlLoopFrequency frequency) noexcept;

    /// @brief Stops the background thread if still running, then destroys.
    ~ControlLoop();

    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;
    ControlLoop(ControlLoop&&) = delete;
    ControlLoop& operator=(ControlLoop&&) = delete;

    /// @brief Executes exactly one control cycle synchronously.
    ///
    /// Reads controller's state(); calls robot.update(period()) if and
    /// only if that state is robot::controller::ControllerState::Moving.
    /// Always increments the cyclesExecuted() count. No heap allocation,
    /// no sleep, no thread — safe and meaningful to call directly from a
    /// test with no background thread ever started.
    void step() noexcept;

    /// @brief Starts a background thread that calls step() in a loop,
    ///        paced to period().
    /// @return Success, or ControlLoopError::AlreadyRunning if a
    ///         background thread from a previous start() is still active.
    [[nodiscard]] std::expected<void, ControlLoopError> start();

    /// @brief Signals the background thread to stop and blocks until it
    ///        has exited (see the bounded-latency note on the class).
    /// @return Success, or ControlLoopError::NotRunning if no background
    ///         thread is currently active.
    [[nodiscard]] std::expected<void, ControlLoopError> stop();

    /// @brief Whether a background thread started by start() is currently active.
    [[nodiscard]] bool isRunning() const noexcept;

    /// @brief The fixed target duration of one cycle, derived from the
    ///        frequency given at construction (e.g. Hz1000 -> 1'000'000ns).
    [[nodiscard]] std::chrono::nanoseconds period() const noexcept;

    /// @brief A snapshot of this ControlLoop's timing statistics.
    ///
    /// Individual ControlLoopMetrics fields are read from independent
    /// atomics, not captured as a single atomic transaction — see
    /// ControlLoopMetrics's documentation for what that means for a
    /// reader on another thread while the loop is running.
    [[nodiscard]] ControlLoopMetrics metrics() const noexcept;

private:
    /// @brief Body of the background thread started by start().
    void runLoop(std::stop_token stopToken);

    /// @brief Atomically updates maxJitterNs_ to std::max(maxJitterNs_, candidate).
    void updateMaxJitter(std::chrono::nanoseconds candidate) noexcept;

    robot::core::Robot& robot_;
    robot::controller::ControllerStateMachine& controller_;
    std::chrono::nanoseconds period_;

    std::atomic<bool> running_{false};
    std::jthread thread_;

    // Incremented by every step() call, direct or threaded — see
    // ControlLoopMetrics::cyclesExecuted.
    std::atomic<std::uint64_t> cyclesExecuted_{0};

    // Populated only by the threaded path (runLoop) — see
    // ControlLoopMetrics::averageCycleTime/maxJitter/deadlineMisses.
    std::atomic<std::uint64_t> threadedCycleCount_{0};
    std::atomic<std::int64_t> totalThreadedCycleTimeNs_{0};
    std::atomic<std::int64_t> maxJitterNs_{0};
    std::atomic<std::uint64_t> deadlineMisses_{0};
};

}  // namespace robot::runtime
