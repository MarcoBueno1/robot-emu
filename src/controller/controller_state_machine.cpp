// src/controller/controller_state_machine.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/controller/controller_state_machine.hpp"

#include <algorithm>
#include <array>

namespace robot::controller {

namespace {

struct Transition {
    ControllerState from;
    ControllerEvent event;
    ControllerState to;
};

// Authoritative transition table — see
// docs/task-briefs/phase-02-controller-state-machine.md section 6.1.
// Deliberately a flat, static constexpr array: at this size (~30 rows) a
// linear scan is simpler, just as fast in practice, and easier to audit
// row-by-row than a map/hash-based lookup.
constexpr std::array<Transition, 26> kTransitionTable{{
    {ControllerState::PowerOff,      ControllerEvent::PowerOn,          ControllerState::Booting},
    {ControllerState::Booting,       ControllerEvent::BootComplete,     ControllerState::Initializing},
    {ControllerState::Initializing,  ControllerEvent::InitComplete,     ControllerState::Idle},

    {ControllerState::Idle,          ControllerEvent::ServoEnable,      ControllerState::ServoOn},
    {ControllerState::ServoOff,      ControllerEvent::ServoEnable,      ControllerState::ServoOn},
    {ControllerState::ServoOn,       ControllerEvent::ServoDisable,     ControllerState::ServoOff},
    {ControllerState::ServoOn,       ControllerEvent::ControllerReady,  ControllerState::Ready},
    {ControllerState::Ready,         ControllerEvent::ServoDisable,     ControllerState::ServoOff},

    {ControllerState::Ready,         ControllerEvent::CommandMove,      ControllerState::Moving},
    {ControllerState::Moving,        ControllerEvent::MotionComplete,   ControllerState::Ready},
    {ControllerState::Moving,        ControllerEvent::CommandPause,     ControllerState::Paused},
    {ControllerState::Paused,        ControllerEvent::CommandResume,    ControllerState::Moving},

    {ControllerState::Moving,        ControllerEvent::CommandStop,      ControllerState::Stopping},
    {ControllerState::Paused,        ControllerEvent::CommandStop,      ControllerState::Stopping},
    {ControllerState::Ready,         ControllerEvent::CommandStop,      ControllerState::Stopping},
    {ControllerState::Stopping,      ControllerEvent::StopComplete,     ControllerState::Ready},

    {ControllerState::ServoOn,       ControllerEvent::EStopTriggered,   ControllerState::EmergencyStop},
    {ControllerState::Ready,         ControllerEvent::EStopTriggered,   ControllerState::EmergencyStop},
    {ControllerState::Moving,        ControllerEvent::EStopTriggered,   ControllerState::EmergencyStop},
    {ControllerState::Paused,        ControllerEvent::EStopTriggered,   ControllerState::EmergencyStop},
    {ControllerState::Stopping,      ControllerEvent::EStopTriggered,   ControllerState::EmergencyStop},
    {ControllerState::EmergencyStop, ControllerEvent::EStopReset,       ControllerState::Ready},

    {ControllerState::Fault,         ControllerEvent::RecoveryStart,    ControllerState::Recovery},
    {ControllerState::Recovery,      ControllerEvent::RecoveryComplete, ControllerState::Ready},

    {ControllerState::Idle,          ControllerEvent::PowerOff,         ControllerState::Shutdown},
    {ControllerState::ServoOff,      ControllerEvent::PowerOff,         ControllerState::Shutdown},
}};

// FaultDetected is reachable from every "live" operational state, and
// PowerOff (shutdown) from a couple more — split into a second table purely
// so each table stays short enough to eyeball against the brief's section
// 6.1 table without horizontal scrolling.
constexpr std::array<Transition, 10> kFaultAndShutdownTable{{
    {ControllerState::Idle,          ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::ServoOff,      ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::ServoOn,       ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::Ready,         ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::Moving,        ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::Paused,        ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::Stopping,      ControllerEvent::FaultDetected,    ControllerState::Fault},
    {ControllerState::EmergencyStop, ControllerEvent::FaultDetected,    ControllerState::Fault},

    {ControllerState::Ready,         ControllerEvent::PowerOff,         ControllerState::Shutdown},
    {ControllerState::Fault,         ControllerEvent::PowerOff,         ControllerState::Shutdown},
}};

[[nodiscard]] const Transition* findTransition(ControllerState from, ControllerEvent event) noexcept {
    auto match = [&](const Transition& t) { return t.from == from && t.event == event; };

    if (auto it = std::find_if(kTransitionTable.begin(), kTransitionTable.end(), match);
        it != kTransitionTable.end()) {
        return &*it;
    }
    if (auto it = std::find_if(kFaultAndShutdownTable.begin(), kFaultAndShutdownTable.end(), match);
        it != kFaultAndShutdownTable.end()) {
        return &*it;
    }
    return nullptr;
}

}  // namespace

ControllerStateMachine::ControllerStateMachine() noexcept = default;

ControllerState ControllerStateMachine::state() const noexcept {
    return state_;
}

bool ControllerStateMachine::canHandle(ControllerEvent event) const noexcept {
    return findTransition(state_, event) != nullptr;
}

std::expected<void, ControllerError> ControllerStateMachine::handleEvent(ControllerEvent event) noexcept {
    const Transition* transition = findTransition(state_, event);
    if (transition == nullptr) {
        return std::unexpected(ControllerError::InvalidTransition);
    }
    state_ = transition->to;
    return {};
}

std::string_view toString(ControllerState state) noexcept {
    switch (state) {
        case ControllerState::PowerOff:      return "PowerOff";
        case ControllerState::Booting:       return "Booting";
        case ControllerState::Initializing:  return "Initializing";
        case ControllerState::Idle:          return "Idle";
        case ControllerState::ServoOff:      return "ServoOff";
        case ControllerState::ServoOn:       return "ServoOn";
        case ControllerState::Ready:         return "Ready";
        case ControllerState::Moving:        return "Moving";
        case ControllerState::Paused:        return "Paused";
        case ControllerState::Stopping:      return "Stopping";
        case ControllerState::EmergencyStop: return "EmergencyStop";
        case ControllerState::Fault:         return "Fault";
        case ControllerState::Recovery:      return "Recovery";
        case ControllerState::Shutdown:      return "Shutdown";
    }
    __builtin_unreachable();
}

std::string_view toString(ControllerEvent event) noexcept {
    switch (event) {
        case ControllerEvent::PowerOn:          return "PowerOn";
        case ControllerEvent::BootComplete:     return "BootComplete";
        case ControllerEvent::InitComplete:     return "InitComplete";
        case ControllerEvent::ServoEnable:      return "ServoEnable";
        case ControllerEvent::ServoDisable:     return "ServoDisable";
        case ControllerEvent::ControllerReady:  return "ControllerReady";
        case ControllerEvent::CommandMove:      return "CommandMove";
        case ControllerEvent::MotionComplete:   return "MotionComplete";
        case ControllerEvent::CommandPause:     return "CommandPause";
        case ControllerEvent::CommandResume:    return "CommandResume";
        case ControllerEvent::CommandStop:      return "CommandStop";
        case ControllerEvent::StopComplete:     return "StopComplete";
        case ControllerEvent::EStopTriggered:   return "EStopTriggered";
        case ControllerEvent::EStopReset:       return "EStopReset";
        case ControllerEvent::FaultDetected:    return "FaultDetected";
        case ControllerEvent::RecoveryStart:    return "RecoveryStart";
        case ControllerEvent::RecoveryComplete: return "RecoveryComplete";
        case ControllerEvent::PowerOff:         return "PowerOff";
    }
    __builtin_unreachable();
}

}  // namespace robot::controller
