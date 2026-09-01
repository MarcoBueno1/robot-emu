// include/robot/server/command_dispatcher.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include "robot/controller/controller_state_machine.hpp"
#include "robot/core/robot.hpp"
#include "robot/protocol/frame.hpp"
#include "robot/safety/emergency_stop_controller.hpp"

namespace robot::server {

/// @brief Given a request Frame, performs the corresponding action against
///        a Robot/ControllerStateMachine/EmergencyStopController and
///        returns an encoded response Frame, following exactly the
///        convention docs/task-briefs/phase-07-cli.md sections 6.2-6.4 define.
///
/// Pure logic — no networking, no threading, no locking. This class does
/// no synchronization of its own; see
/// docs/task-briefs/server-integration.md section 6.2 for the concurrency
/// design a caller (apps/robot-emulator/main.cpp) is responsible for:
/// every call to dispatch() must be made while holding the same mutex that
/// guards the shared Robot/ControllerStateMachine against the control
/// thread's concurrent ControlLoop::step() calls.
class CommandDispatcher {
public:
    /// @param robot, controller, estop Borrowed, must outlive this dispatcher.
    CommandDispatcher(robot::core::Robot& robot, robot::controller::ControllerStateMachine& controller,
                       robot::safety::EmergencyStopController& estop) noexcept;

    /// @brief Handles one request.
    ///
    /// Unrecognized/unsupported CommandTypes (anything beyond GetStatus,
    /// Enable, Disable, Home, MoveJoint, Stop, EmergencyStop — see
    /// docs/task-briefs/server-integration.md Non-Goals) get a
    /// well-formed error response echoing the request's type, never a
    /// crash or an unanswered request.
    [[nodiscard]] robot::protocol::Frame dispatch(const robot::protocol::Frame& request);

private:
    [[nodiscard]] robot::protocol::Frame handleGetStatus();
    [[nodiscard]] robot::protocol::Frame handleEnable();
    [[nodiscard]] robot::protocol::Frame handleDisable();
    [[nodiscard]] robot::protocol::Frame handleHome();
    [[nodiscard]] robot::protocol::Frame handleMoveJoint(const robot::protocol::Frame& request);
    [[nodiscard]] robot::protocol::Frame handleStop();
    [[nodiscard]] robot::protocol::Frame handleEmergencyStop();

    [[nodiscard]] robot::protocol::Frame okResponse(robot::protocol::CommandType type,
                                                      std::vector<std::byte> extra = {}) const;
    [[nodiscard]] robot::protocol::Frame errorResponse(robot::protocol::CommandType type) const;

    robot::core::Robot& robot_;
    robot::controller::ControllerStateMachine& controller_;
    robot::safety::EmergencyStopController& estop_;
};

}  // namespace robot::server
