// src/safety/emergency_stop_controller.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/safety/emergency_stop_controller.hpp"

namespace robot::safety {

EmergencyStopController::EmergencyStopController(robot::controller::ControllerStateMachine& controller,
                                                   std::span<robot::hardware::VirtualBrake> brakes) noexcept
    : controller_(controller), brakes_(brakes) {}

void EmergencyStopController::trigger() noexcept {
    // Unconditional, regardless of what happens next — physical safety
    // must never be gated on the state machine's bookkeeping succeeding.
    for (auto& brake : brakes_) {
        brake.engage();
    }

    // Result intentionally discarded: whether or not this particular
    // transition was legal from the controller's current state, the
    // brakes above are already engaged. A caller wanting to know whether
    // the state machine itself accepted the transition can inspect
    // controller state directly; trigger()'s contract is about the
    // brakes, not the transition.
    static_cast<void>(controller_.handleEvent(robot::controller::ControllerEvent::EStopTriggered));
}

std::expected<void, robot::controller::ControllerError> EmergencyStopController::reset() noexcept {
    return controller_.handleEvent(robot::controller::ControllerEvent::EStopReset);
}

}  // namespace robot::safety
