// include/robot/safety/watchdog_error.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once

namespace robot::safety {

/// @brief Errors returned by fallible lifecycle operations on Watchdog.
///
/// Used exclusively with std::expected — no exceptions on the control
/// path (see CONTRIBUTING.md and docs/architecture.md section 3.15).
enum class WatchdogError {
    AlreadyRunning,  ///< start() called while the watchdog is already running.
    NotRunning,      ///< stop() called while the watchdog is not running.
};

}  // namespace robot::safety
