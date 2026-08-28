// include/robot/controller/controller_state_machine.hpp
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
#include "robot/controller/controller_error.hpp"
#include "robot/controller/controller_event.hpp"
#include "robot/controller/controller_state.hpp"

namespace robot::controller {

/// @brief Explicit, table-driven state machine for the controller.
///
/// ControllerStateMachine is a pure, synchronous state container: it reads
/// no clock, does no I/O, owns no robot::core::Robot, and spawns no thread.
/// It is driven entirely by explicit handleEvent() calls; wiring real
/// triggers to those calls (cycle completion, network commands, a
/// watchdog) is the job of later phases (3, 6, 8 respectively).
///
/// The full transition table is authoritative in
/// docs/task-briefs/phase-02-controller-state-machine.md section 6.1 — this
/// class implements exactly that table, unconditionally (no guards in this
/// phase; see the brief's Non-Goals for why).
///
/// Thread-safety: none — like Joint and Robot in Phase 1, no internal
/// synchronization is done by design. A single owner thread is expected to
/// call handleEvent().
class ControllerStateMachine {
public:
    /// @brief Constructs a state machine starting in ControllerState::PowerOff.
    ControllerStateMachine() noexcept;

    /// @brief The current state.
    [[nodiscard]] ControllerState state() const noexcept;

    /// @brief Checks whether event has a transition table entry from the
    ///        current state, without applying it.
    /// @param event Event to check.
    /// @return true if handleEvent(event) would succeed right now.
    [[nodiscard]] bool canHandle(ControllerEvent event) const noexcept;

    /// @brief Applies event if it is legal from the current state.
    ///
    /// On success, state() reflects the new state immediately afterward.
    /// On failure, state() is left completely unchanged.
    /// @param event Event to apply.
    /// @return Success, or ControllerError::InvalidTransition if
    ///         (state(), event) has no entry in the transition table.
    [[nodiscard]] std::expected<void, ControllerError> handleEvent(ControllerEvent event) noexcept;

private:
    ControllerState state_ = ControllerState::PowerOff;
};

}  // namespace robot::controller
