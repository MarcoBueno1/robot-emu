// include/robot/runtime/control_loop_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::runtime {

/// @brief Errors returned by fallible lifecycle operations on ControlLoop.
///
/// Used exclusively with std::expected — no exceptions are thrown on the
/// control path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class ControlLoopError {
    AlreadyRunning,  ///< start() called while the loop is already running.
    NotRunning,      ///< stop() called while the loop is not running.
};

}  // namespace robot::runtime
