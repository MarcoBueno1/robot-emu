// include/robot/fault/fault_type.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <string_view>

namespace robot::fault {

/// @brief The fault catalog from docs/architecture.md section 3.9.
///
/// Deliberately independent of any other module's enums (e.g.
/// robot::hardware::BrakeState) — a "brake failure" fault and a
/// VirtualBrake's current engaged/released position are different
/// concepts. See docs/task-briefs/phase-10-fault-injection.md Notes.
enum class FaultType {
    EncoderFailure,
    MotorFailure,
    OverCurrent,
    OverTemperature,
    CommunicationTimeout,
    PositionError,
    VelocityError,
    BrakeFailure,
    LimitSwitch,
    EmergencyStop,
    PowerFailure,
};

/// @brief Human-readable name for a FaultType, e.g. for logging.
[[nodiscard]] std::string_view toString(FaultType type) noexcept;

}  // namespace robot::fault
