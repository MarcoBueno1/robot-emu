// tests/server/command_dispatcher_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <array>
#include "robot/cli/move_joint_payload.hpp"
#include "robot/cli/response.hpp"
#include "robot/cli/status_payload.hpp"
#include "robot/server/command_dispatcher.hpp"

using namespace robot::server;
using robot::cli::JointStatus;
using robot::cli::MoveJointPayload;
using robot::cli::ResponseStatus;
using robot::cli::StatusPayload;
using robot::controller::ControllerState;
using robot::controller::ControllerStateMachine;
using robot::core::Joint;
using robot::core::JointLimits;
using robot::core::Robot;
using robot::core::RobotConfig;
using robot::hardware::VirtualBrake;
using robot::protocol::CommandType;
using robot::protocol::Frame;
using robot::safety::EmergencyStopController;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -3.14,
        .max_position     =  3.14,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

Robot sixJointRobot() {
    RobotConfig config{.name = "test", .joints = std::vector<JointLimits>(6, defaultLimits())};
    return Robot::create(config).value();
}

ResponseStatus statusOf(const Frame& frame) {
    return static_cast<ResponseStatus>(frame.payload.at(0));
}

}  // namespace

class CommandDispatcherTest : public ::testing::Test {
protected:
    Robot robot = sixJointRobot();
    ControllerStateMachine controller;
    std::array<VirtualBrake, 6> brakes;
    EmergencyStopController estop{controller, brakes};
    CommandDispatcher dispatcher{robot, controller, estop};
};

TEST_F(CommandDispatcherTest, UnrecognizedCommandReturnsErrorEchoingType) {
    Frame request;
    request.type = CommandType::GetIO;  // not one of the 7 supported commands

    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(response.type, CommandType::GetIO);
    EXPECT_EQ(statusOf(response), ResponseStatus::Error);
}

TEST_F(CommandDispatcherTest, EnableEnablesEveryJointAndReachesReady) {
    for (auto event : {robot::controller::ControllerEvent::PowerOn, robot::controller::ControllerEvent::BootComplete,
                        robot::controller::ControllerEvent::InitComplete}) {
        ASSERT_TRUE(controller.handleEvent(event).has_value());
    }

    Frame request;
    request.type = CommandType::Enable;
    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    EXPECT_EQ(controller.state(), ControllerState::Ready);
    for (const auto& joint : robot.joints()) {
        EXPECT_TRUE(joint.enabled());
    }
}

TEST_F(CommandDispatcherTest, DisableDisablesEveryJoint) {
    for (auto& joint : robot.joints()) {
        joint.enable();
    }

    Frame request;
    request.type = CommandType::Disable;
    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    for (const auto& joint : robot.joints()) {
        EXPECT_FALSE(joint.enabled());
    }
}

TEST_F(CommandDispatcherTest, HomeHomesEveryJoint) {
    for (auto& joint : robot.joints()) {
        joint.enable();
        ASSERT_TRUE(joint.setTargetPosition(1.0).has_value());
        for (int i = 0; i < 2000; ++i) {
            joint.update(std::chrono::milliseconds(1));
        }
    }

    Frame request;
    request.type = CommandType::Home;
    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    for (const auto& joint : robot.joints()) {
        EXPECT_TRUE(joint.homed());
        EXPECT_DOUBLE_EQ(joint.position(), 0.0);
    }
}

TEST_F(CommandDispatcherTest, MoveJointSetsTargetAndEntersMovingFromReady) {
    for (auto event : {robot::controller::ControllerEvent::PowerOn, robot::controller::ControllerEvent::BootComplete,
                        robot::controller::ControllerEvent::InitComplete,
                        robot::controller::ControllerEvent::ServoEnable,
                        robot::controller::ControllerEvent::ControllerReady}) {
        ASSERT_TRUE(controller.handleEvent(event).has_value());
    }
    for (auto& joint : robot.joints()) {
        joint.enable();
    }

    Frame request;
    request.type = CommandType::MoveJoint;
    request.payload = robot::cli::encodeMoveJointPayload(MoveJointPayload{2, 1.5});

    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    EXPECT_EQ(controller.state(), ControllerState::Moving);
}

TEST_F(CommandDispatcherTest, MoveJointWithTruncatedPayloadReturnsError) {
    Frame request;
    request.type = CommandType::MoveJoint;
    request.payload = {std::byte{0}};  // too short to decode

    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Error);
}

TEST_F(CommandDispatcherTest, MoveJointWithOutOfRangeJointIndexReturnsError) {
    Frame request;
    request.type = CommandType::MoveJoint;
    request.payload = robot::cli::encodeMoveJointPayload(MoveJointPayload{200, 0.5});

    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Error);
}

TEST_F(CommandDispatcherTest, StopTransitionsBackToReady) {
    for (auto event : {robot::controller::ControllerEvent::PowerOn, robot::controller::ControllerEvent::BootComplete,
                        robot::controller::ControllerEvent::InitComplete,
                        robot::controller::ControllerEvent::ServoEnable,
                        robot::controller::ControllerEvent::ControllerReady,
                        robot::controller::ControllerEvent::CommandMove}) {
        ASSERT_TRUE(controller.handleEvent(event).has_value());
    }
    ASSERT_EQ(controller.state(), ControllerState::Moving);

    Frame request;
    request.type = CommandType::Stop;
    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    EXPECT_EQ(controller.state(), ControllerState::Ready);
}

TEST_F(CommandDispatcherTest, EmergencyStopEngagesBrakesAndTransitionsState) {
    for (auto& brake : brakes) {
        brake.release();
    }
    for (auto event : {robot::controller::ControllerEvent::PowerOn, robot::controller::ControllerEvent::BootComplete,
                        robot::controller::ControllerEvent::InitComplete,
                        robot::controller::ControllerEvent::ServoEnable,
                        robot::controller::ControllerEvent::ControllerReady}) {
        ASSERT_TRUE(controller.handleEvent(event).has_value());
    }

    Frame request;
    request.type = CommandType::EmergencyStop;
    auto response = dispatcher.dispatch(request);

    EXPECT_EQ(statusOf(response), ResponseStatus::Ok);
    EXPECT_EQ(controller.state(), ControllerState::EmergencyStop);
    for (const auto& brake : brakes) {
        EXPECT_TRUE(brake.isEngaged());
    }
}

TEST_F(CommandDispatcherTest, GetStatusReflectsLiveRobotAndControllerState) {
    for (auto event : {robot::controller::ControllerEvent::PowerOn, robot::controller::ControllerEvent::BootComplete,
                        robot::controller::ControllerEvent::InitComplete}) {
        ASSERT_TRUE(controller.handleEvent(event).has_value());
    }

    Frame request;
    request.type = CommandType::GetStatus;
    auto response = dispatcher.dispatch(request);

    ASSERT_EQ(statusOf(response), ResponseStatus::Ok);
    ASSERT_EQ(response.type, CommandType::GetStatus);

    std::span<const std::byte> statusBytes(response.payload.data() + 1, response.payload.size() - 1);
    auto status = robot::cli::decodeStatusPayload(statusBytes);
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->controllerStateName, "Idle");
    EXPECT_EQ(status->joints.size(), 6u);
}
