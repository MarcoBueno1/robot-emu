// include/robot/hardware/hardware_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::hardware {

/// @brief Errors returned by fallible operations across every type in this
///        module (VirtualMotor, VirtualEncoder, DigitalIO, AnalogIO).
///
/// One shared enum rather than a near-duplicate per type — the same
/// precedent robot::motion set in Phase 4 for reusing robot::core::JointError.
/// Used exclusively with std::expected — no exceptions on the control path
/// (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class HardwareError {
    InvalidConfiguration,  ///< Bad constructor/create() parameters.
    OutOfRange,             ///< A commanded value, or a channel index, is out of bounds.
};

}  // namespace robot::hardware
