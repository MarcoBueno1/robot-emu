// include/robot/controller/controller_state.hpp
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

namespace robot::controller {

/// @brief Formal states of the controller state machine.
///
/// See docs/architecture.md section 3.4 for the source diagram and
/// docs/task-briefs/phase-02-controller-state-machine.md section 6.1 for
/// the authoritative, unambiguous transition table implemented by
/// ControllerStateMachine.
enum class ControllerState {
    PowerOff,       ///< Initial state. No power, nothing initialized.
    Booting,        ///< Power applied, firmware/runtime booting.
    Initializing,   ///< Boot complete, running startup self-checks.
    Idle,           ///< Initialized, servos never yet enabled.
    ServoOff,       ///< Servos explicitly disabled after having been on.
    ServoOn,        ///< Servos enabled, not yet confirmed ready.
    Ready,          ///< Servos enabled and ready to accept motion commands.
    Moving,         ///< Executing a motion command.
    Paused,         ///< Motion suspended, resumable.
    Stopping,       ///< Controlled deceleration/stop in progress.
    EmergencyStop,  ///< Emergency stop latched — only EStopReset or FaultDetected leave this state.
    Fault,          ///< A fault has been detected.
    Recovery,       ///< Recovering from a fault.
    Shutdown,       ///< Terminal state — no events are accepted from here.
};

/// @brief Human-readable name for a ControllerState, e.g. for logging.
/// @return A string_view into static storage (a string literal) — never
///         owning, so this function performs no allocation and is noexcept.
[[nodiscard]] std::string_view toString(ControllerState state) noexcept;

}  // namespace robot::controller
