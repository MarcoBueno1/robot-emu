// include/robot/safety/limit_monitor.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <vector>
#include "robot/core/robot.hpp"
#include "robot/safety/limit_violation.hpp"

namespace robot::safety {

/// @brief Proactively flags joints within a configurable margin of their
///        position limits.
///
/// This is a *soft*, observational check, independent of Joint's own hard
/// clamp (Phase 1) — the same defense-in-depth idea as a real robot's
/// software position limits being checked separately from its physical
/// hard stops. Because Joint's own update() already makes it structurally
/// hard to end up outside [min_position, max_position] at all, a
/// LimitMonitor mostly matters as a *proactive* early-warning check (catch
/// a joint approaching a bound before it gets there), not as the only line
/// of defense against actually exceeding one.
class LimitMonitor {
public:
    /// @param marginRadians How close to min_position/max_position counts
    ///        as a violation. Must be >= 0.
    explicit LimitMonitor(double marginRadians) noexcept;

    /// @brief Scans every joint in robot.
    /// @return One LimitViolation per joint currently within
    ///         marginRadians of either bound, in robot.joints() order
    ///         (J1..JN). Empty if none are.
    [[nodiscard]] std::vector<LimitViolation> check(const robot::core::Robot& robot) const;

private:
    double marginRadians_;
};

}  // namespace robot::safety
