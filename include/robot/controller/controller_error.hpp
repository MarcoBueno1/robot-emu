// include/robot/controller/controller_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::controller {

/// @brief Errors returned by fallible operations on ControllerStateMachine.
///
/// Used exclusively with std::expected — no exceptions are thrown on the
/// control path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class ControllerError {
    InvalidTransition,  ///< No transition table entry for (current state, event).
};

}  // namespace robot::controller
