// include/robot/core/joint_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::core {

/// @brief Errors returned by fallible operations on Joint, Robot, and their
///        configuration types. Used exclusively with std::expected — no
///        exceptions are thrown on the control path (see CONTRIBUTING.md).
enum class JointError {
    PositionOutOfRange,      ///< Requested/configured position is outside [min_position, max_position].
    VelocityOutOfRange,      ///< Requested/configured velocity magnitude exceeds max_velocity.
    AccelerationOutOfRange,  ///< Requested/configured acceleration magnitude exceeds max_acceleration.
    JointDisabled,           ///< Operation requires the joint to be enabled first.
    InvalidConfiguration,    ///< Configuration is internally inconsistent (e.g. min >= max, zero joints).
};

}
