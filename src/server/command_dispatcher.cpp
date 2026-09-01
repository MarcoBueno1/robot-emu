// src/server/command_dispatcher.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/server/command_dispatcher.hpp"

#include "robot/cli/move_joint_payload.hpp"
#include "robot/cli/response.hpp"
#include "robot/cli/status_payload.hpp"
#include "robot/controller/controller_state.hpp"

namespace robot::server {

using robot::controller::ControllerEvent;
using robot::controller::ControllerState;
using robot::protocol::CommandType;
using robot::protocol::Frame;

namespace {
// This server hardcodes a single, fixed scenario (see
// docs/task-briefs/server-integration.md Non-Goals — no declarative config
// file yet), so a name reported in GetStatus responses is a compile-time
// constant here rather than something read back from Robot (which, like
// JointLimits before Phase 8's Joint::limits() addition, has no public
// name() accessor — not worth adding one for a single hardcoded label).
constexpr std::string_view kRobotName = "RE-6AXIS";
}  // namespace

CommandDispatcher::CommandDispatcher(robot::core::Robot& robot,
                                      robot::controller::ControllerStateMachine& controller,
                                      robot::safety::EmergencyStopController& estop) noexcept
    : robot_(robot), controller_(controller), estop_(estop) {}

Frame CommandDispatcher::dispatch(const Frame& request) {
    switch (request.type) {
        case CommandType::GetStatus:     return handleGetStatus();
        case CommandType::Enable:        return handleEnable();
        case CommandType::Disable:       return handleDisable();
        case CommandType::Home:          return handleHome();
        case CommandType::MoveJoint:     return handleMoveJoint(request);
        case CommandType::Stop:          return handleStop();
        case CommandType::EmergencyStop: return handleEmergencyStop();
        default:
            // Every CommandType this server doesn't implement (MoveLinear,
            // GetPosition, GetIO, SetIO, ResetFault, InjectFault, Connect)
            // — see Non-Goals. A well-formed error, never a crash.
            return errorResponse(request.type);
    }
}

Frame CommandDispatcher::handleGetStatus() {
    robot::cli::StatusPayload status;
    status.robotName = std::string(kRobotName);
    status.controllerStateName = std::string(robot::controller::toString(controller_.state()));
    status.joints.reserve(robot_.jointCount());
    for (const auto& joint : robot_.joints()) {
        status.joints.push_back(robot::cli::JointStatus{joint.position(), joint.velocity()});
    }
    return okResponse(CommandType::GetStatus, robot::cli::encodeStatusPayload(status));
}

Frame CommandDispatcher::handleEnable() {
    for (auto& joint : robot_.joints()) {
        joint.enable();
    }
    static_cast<void>(controller_.handleEvent(ControllerEvent::ServoEnable));
    static_cast<void>(controller_.handleEvent(ControllerEvent::ControllerReady));
    return okResponse(CommandType::Enable);
}

Frame CommandDispatcher::handleDisable() {
    for (auto& joint : robot_.joints()) {
        joint.disable();
    }
    static_cast<void>(controller_.handleEvent(ControllerEvent::ServoDisable));
    return okResponse(CommandType::Disable);
}

Frame CommandDispatcher::handleHome() {
    for (auto& joint : robot_.joints()) {
        joint.home();
    }
    // No controller state transition — homing is joint-level only; there
    // is no Home ControllerEvent (see the task brief's section 6.3).
    return okResponse(CommandType::Home);
}

Frame CommandDispatcher::handleMoveJoint(const Frame& request) {
    auto payload = robot::cli::decodeMoveJointPayload(request.payload);
    if (!payload.has_value()) {
        return errorResponse(CommandType::MoveJoint);
    }
    if (payload->jointIndex >= robot_.jointCount()) {
        return errorResponse(CommandType::MoveJoint);
    }

    auto set = robot_.joint(payload->jointIndex).setTargetPosition(payload->targetRadians);
    if (!set.has_value()) {
        return errorResponse(CommandType::MoveJoint);
    }

    if (controller_.state() == ControllerState::Ready) {
        static_cast<void>(controller_.handleEvent(ControllerEvent::CommandMove));
    }

    return okResponse(CommandType::MoveJoint);
}

Frame CommandDispatcher::handleStop() {
    static_cast<void>(controller_.handleEvent(ControllerEvent::CommandStop));
    static_cast<void>(controller_.handleEvent(ControllerEvent::StopComplete));
    return okResponse(CommandType::Stop);
}

Frame CommandDispatcher::handleEmergencyStop() {
    estop_.trigger();
    return okResponse(CommandType::EmergencyStop);
}

Frame CommandDispatcher::okResponse(CommandType type, std::vector<std::byte> extra) const {
    Frame response;
    response.type = type;
    response.payload.push_back(static_cast<std::byte>(robot::cli::ResponseStatus::Ok));
    response.payload.insert(response.payload.end(), extra.begin(), extra.end());
    return response;
}

Frame CommandDispatcher::errorResponse(CommandType type) const {
    Frame response;
    response.type = type;
    response.payload.push_back(static_cast<std::byte>(robot::cli::ResponseStatus::Error));
    return response;
}

}  // namespace robot::server
