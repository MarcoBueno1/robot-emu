// include/robot/fault/active_fault.hpp
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
#include <optional>
#include "robot/fault/fault_type.hpp"

namespace robot::fault {

/// @brief One currently-active fault, as tracked by FaultInjector.
struct ActiveFault {
    FaultType type;
    /// nullopt for a robot-wide fault (e.g. PowerFailure); set for a
    /// fault scoped to one joint (e.g. EncoderFailure on joint 2, matching
    /// section 3.9's `INJECT_FAULT type=ENCODER_FAILURE joint=2` example).
    std::optional<std::size_t> jointIndex;

    [[nodiscard]] friend bool operator==(const ActiveFault&, const ActiveFault&) = default;
};

}  // namespace robot::fault
