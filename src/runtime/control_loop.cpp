// src/runtime/control_loop.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/runtime/control_loop.hpp"

namespace robot::runtime {

namespace {

// Exact for every ControlLoopFrequency enumerator: 1e9 is evenly divisible
// by 500, 1000, 2000, 5000, and 10000, so no floating-point rounding is
// ever involved here.
[[nodiscard]] std::chrono::nanoseconds periodFor(ControlLoopFrequency frequency) noexcept {
    const auto hz = static_cast<std::int64_t>(frequency);
    return std::chrono::nanoseconds(1'000'000'000LL / hz);
}

}  // namespace

ControlLoop::ControlLoop(robot::core::Robot& robot,
                          robot::controller::ControllerStateMachine& controller,
                          ControlLoopFrequency frequency) noexcept
    : robot_(robot), controller_(controller), period_(periodFor(frequency)) {}

ControlLoop::~ControlLoop() {
    if (running_.load(std::memory_order_acquire)) {
        running_.store(false, std::memory_order_release);
        thread_.request_stop();
        // thread_'s own destructor joins automatically once this destructor
        // body finishes and members are torn down — request_stop() here
        // just lets the loop notice sooner than waiting for that implicit
        // join to call it. See the class-level "known bounded latency" note.
    }
}

void ControlLoop::step() noexcept {
    if (controller_.state() == robot::controller::ControllerState::Moving) {
        robot_.update(period_);
    }
    cyclesExecuted_.fetch_add(1, std::memory_order_relaxed);
}

std::expected<void, ControlLoopError> ControlLoop::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return std::unexpected(ControlLoopError::AlreadyRunning);
    }
    thread_ = std::jthread([this](std::stop_token stopToken) { runLoop(stopToken); });
    return {};
}

std::expected<void, ControlLoopError> ControlLoop::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
        return std::unexpected(ControlLoopError::NotRunning);
    }
    thread_.request_stop();
    if (thread_.joinable()) {
        thread_.join();
    }
    return {};
}

bool ControlLoop::isRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
}

std::chrono::nanoseconds ControlLoop::period() const noexcept {
    return period_;
}

ControlLoopMetrics ControlLoop::metrics() const noexcept {
    ControlLoopMetrics snapshot;
    snapshot.cyclesExecuted = cyclesExecuted_.load(std::memory_order_relaxed);
    snapshot.deadlineMisses = deadlineMisses_.load(std::memory_order_relaxed);

    const auto threadedCount = threadedCycleCount_.load(std::memory_order_relaxed);
    const auto totalNs = totalThreadedCycleTimeNs_.load(std::memory_order_relaxed);
    snapshot.averageCycleTime = threadedCount > 0
        ? std::chrono::nanoseconds(totalNs / static_cast<std::int64_t>(threadedCount))
        : std::chrono::nanoseconds(0);

    snapshot.maxJitter = std::chrono::nanoseconds(maxJitterNs_.load(std::memory_order_relaxed));
    return snapshot;
}

void ControlLoop::updateMaxJitter(std::chrono::nanoseconds candidate) noexcept {
    auto candidateNs = candidate.count();
    auto current = maxJitterNs_.load(std::memory_order_relaxed);
    while (candidateNs > current &&
           !maxJitterNs_.compare_exchange_weak(current, candidateNs, std::memory_order_relaxed)) {
        // current is refreshed by compare_exchange_weak on failure; retry.
    }
}

void ControlLoop::runLoop(std::stop_token stopToken) {
    using Clock = std::chrono::steady_clock;

    auto previousCycleStart = Clock::now();
    bool firstCycle = true;

    while (!stopToken.stop_requested()) {
        const auto cycleStart = Clock::now();

        step();

        threadedCycleCount_.fetch_add(1, std::memory_order_relaxed);
        const auto workDuration = Clock::now() - cycleStart;
        totalThreadedCycleTimeNs_.fetch_add(
            std::chrono::duration_cast<std::chrono::nanoseconds>(workDuration).count(),
            std::memory_order_relaxed);

        if (!firstCycle) {
            const auto actualPeriod = cycleStart - previousCycleStart;
            const auto deviation = actualPeriod > period_ ? actualPeriod - period_ : period_ - actualPeriod;
            updateMaxJitter(std::chrono::duration_cast<std::chrono::nanoseconds>(deviation));
        }
        firstCycle = false;
        previousCycleStart = cycleStart;

        const auto nextDeadline = cycleStart + period_;
        if (Clock::now() >= nextDeadline) {
            // No slack left: this cycle's own work (plus bookkeeping)
            // already consumed the full period. Don't try to catch up by
            // running extra cycles back-to-back — proceed immediately to
            // the next cycle instead, same as if it had just come due.
            deadlineMisses_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        std::this_thread::sleep_until(nextDeadline);
    }
}

}  // namespace robot::runtime
