// include/robot/core/clock.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <chrono>

namespace robot::core {

/// @brief Minimal time-source abstraction.
///
/// Allows swapping in a manual clock for deterministic tests (Phase 3+)
/// without touching Robot/Joint. Neither Robot nor Joint read a Clock
/// internally in Phase 1 — dt is always passed in explicitly; Clock exists
/// here so the future control loop (Phase 3) has a ready-made abstraction.
class Clock {
public:
    virtual ~Clock() = default;

    /// @brief Returns the current time according to this clock.
    [[nodiscard]] virtual std::chrono::steady_clock::time_point now() const noexcept = 0;
};

/// @brief Production clock — wraps std::chrono::steady_clock::now().
class SteadyClock final : public Clock {
public:
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override {
        return std::chrono::steady_clock::now();
    }
};

/// @brief Test clock with no reliance on real sleeps.
///
/// Used in control-loop tests (Phase 3): time only moves when advance() is
/// called, making timing-dependent tests instantaneous and deterministic.
class ManualClock final : public Clock {
public:
    [[nodiscard]] std::chrono::steady_clock::time_point now() const noexcept override {
        return now_;
    }

    /// @brief Moves this clock's current time forward by delta.
    /// @param delta Amount of (simulated) time to advance, must be >= 0.
    void advance(std::chrono::nanoseconds delta) noexcept { now_ += delta; }

private:
    std::chrono::steady_clock::time_point now_{};
};

}
