// include/robot/safety/emergency_stop_controller.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include <span>
#include "robot/controller/controller_error.hpp"
#include "robot/controller/controller_state_machine.hpp"
#include "robot/hardware/virtual_brake.hpp"

namespace robot::safety {

/// @brief Wires an emergency stop to both a ControllerStateMachine and a
///        set of VirtualBrakes.
///
/// This is the first real caller of
/// robot::controller::ControllerStateMachine::handleEvent(EStopTriggered/
/// EStopReset) (Phase 2 built the state but nothing triggered it), and the
/// first real caller of robot::hardware::VirtualBrake::engage() at the
/// "robot-wide, unconditional" level Phase 5 anticipated.
class EmergencyStopController {
public:
    /// @param controller Borrowed, must outlive this controller.
    /// @param brakes Borrowed, must outlive this controller. Need not
    ///        correspond 1:1 to any particular Robot's joints — this
    ///        phase does not assume or enforce that correspondence (see
    ///        the task brief's Non-Goals on why no unified "physical
    ///        robot" aggregate exists yet).
    EmergencyStopController(robot::controller::ControllerStateMachine& controller,
                             std::span<robot::hardware::VirtualBrake> brakes) noexcept;

    /// @brief Engages every brake in brakes, unconditionally, THEN
    ///        attempts controller.handleEvent(EStopTriggered).
    ///
    /// Brakes are engaged regardless of whether the state transition
    /// itself succeeds (e.g. if the controller was already PowerOff) —
    /// physical safety must never be gated on a state machine's
    /// bookkeeping succeeding first.
    void trigger() noexcept;

    /// @brief Attempts controller.handleEvent(EStopReset).
    ///
    /// Does not touch any brake — release is always a deliberate, separate
    /// action a caller takes explicitly, never an automatic side effect of
    /// this call.
    [[nodiscard]] std::expected<void, robot::controller::ControllerError> reset() noexcept;

private:
    robot::controller::ControllerStateMachine& controller_;
    std::span<robot::hardware::VirtualBrake> brakes_;
};

}  // namespace robot::safety
