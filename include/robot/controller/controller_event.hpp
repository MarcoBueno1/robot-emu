// include/robot/controller/controller_event.hpp
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

/// @brief Events that drive transitions in ControllerStateMachine.
///
/// This phase defines the events themselves but nothing that emits them —
/// wiring real triggers (network commands, a watchdog, cycle completion)
/// happens in later phases (6, 8, 3 respectively). FaultDetected carries no
/// payload in this phase; a concrete fault reason/code is Phase 10's job.
enum class ControllerEvent {
    PowerOn,            ///< PowerOff -> Booting.
    BootComplete,       ///< Booting -> Initializing.
    InitComplete,       ///< Initializing -> Idle.
    ServoEnable,        ///< Idle/ServoOff -> ServoOn.
    ServoDisable,       ///< ServoOn/Ready -> ServoOff.
    ControllerReady,    ///< ServoOn -> Ready.
    CommandMove,        ///< Ready -> Moving.
    MotionComplete,     ///< Moving -> Ready.
    CommandPause,       ///< Moving -> Paused.
    CommandResume,      ///< Paused -> Moving.
    CommandStop,        ///< Ready/Moving/Paused -> Stopping.
    StopComplete,       ///< Stopping -> Ready.
    EStopTriggered,     ///< ServoOn/Ready/Moving/Paused/Stopping -> EmergencyStop.
    EStopReset,         ///< EmergencyStop -> Ready.
    FaultDetected,      ///< Any live operational state -> Fault.
    RecoveryStart,      ///< Fault -> Recovery.
    RecoveryComplete,   ///< Recovery -> Ready.
    PowerOff,           ///< Idle/ServoOff/Ready/Fault -> Shutdown.
};

/// @brief Human-readable name for a ControllerEvent, e.g. for logging.
/// @return A string_view into static storage (a string literal) — never
///         owning, so this function performs no allocation and is noexcept.
[[nodiscard]] std::string_view toString(ControllerEvent event) noexcept;

}  // namespace robot::controller
