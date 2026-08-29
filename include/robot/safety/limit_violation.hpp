// include/robot/safety/limit_violation.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstddef>

namespace robot::safety {

/// @brief One joint found within LimitMonitor's configured margin of
///        either position bound.
struct LimitViolation {
    std::size_t jointIndex;
    double position;
    /// Always >= 0 — a proximity magnitude, not a signed direction. See
    /// docs/task-briefs/phase-08-safety.md section 8 for why this is
    /// deliberately not directional.
    double distanceToNearestLimit;
};

}  // namespace robot::safety
