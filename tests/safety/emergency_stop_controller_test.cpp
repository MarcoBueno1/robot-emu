// tests/safety/emergency_stop_controller_test.cpp
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
#include "robot/safety/emergency_stop_controller.hpp"

using namespace robot::safety;
using robot::controller::ControllerEvent;
using robot::controller::ControllerState;
using robot::controller::ControllerStateMachine;
using robot::hardware::BrakeState;
using robot::hardware::VirtualBrake;

namespace {
ControllerStateMachine readyMachine() {
    ControllerStateMachine sm;
    for (auto event : {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                        ControllerEvent::ServoEnable, ControllerEvent::ControllerReady}) {
        [[maybe_unused]] auto result = sm.handleEvent(event);
    }
    return sm;
}
}  // namespace

TEST(EmergencyStopController, TriggerEngagesEveryBrakeAndMovesToEmergencyStop) {
    auto controller = readyMachine();
    ASSERT_EQ(controller.state(), ControllerState::Ready);

    std::array<VirtualBrake, 3> brakes;
    for (auto& brake : brakes) {
        brake.release();  // start released, so engagement is actually observable
    }

    EmergencyStopController estop(controller, brakes);
    estop.trigger();

    for (const auto& brake : brakes) {
        EXPECT_EQ(brake.state(), BrakeState::Engaged);
    }
    EXPECT_EQ(controller.state(), ControllerState::EmergencyStop);
}

TEST(EmergencyStopController, TriggerEngagesBrakesEvenWhenTransitionIsIllegal) {
    ControllerStateMachine controller;  // PowerOff — EStopTriggered has no table entry here
    ASSERT_EQ(controller.state(), ControllerState::PowerOff);

    std::array<VirtualBrake, 2> brakes;
    for (auto& brake : brakes) {
        brake.release();
    }

    EmergencyStopController estop(controller, brakes);
    estop.trigger();

    // Brakes must be engaged regardless of the (here, illegal) transition.
    for (const auto& brake : brakes) {
        EXPECT_EQ(brake.state(), BrakeState::Engaged);
    }
    // The state machine itself correctly rejected the transition — trigger()
    // does not force an invalid state, it only guarantees the brakes.
    EXPECT_EQ(controller.state(), ControllerState::PowerOff);
}

TEST(EmergencyStopController, ResetReturnsControllerToReadyAndLeavesBrakesUnchanged) {
    auto controller = readyMachine();
    std::array<VirtualBrake, 2> brakes;
    EmergencyStopController estop(controller, brakes);
    estop.trigger();
    ASSERT_EQ(controller.state(), ControllerState::EmergencyStop);
    for (const auto& brake : brakes) {
        ASSERT_EQ(brake.state(), BrakeState::Engaged);
    }

    auto result = estop.reset();

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(controller.state(), ControllerState::Ready);
    for (const auto& brake : brakes) {
        EXPECT_EQ(brake.state(), BrakeState::Engaged) << "reset() must not touch brakes";
    }
}

TEST(EmergencyStopController, ResetFailsWhenNotInEmergencyStop) {
    auto controller = readyMachine();
    std::array<VirtualBrake, 1> brakes;
    EmergencyStopController estop(controller, brakes);

    auto result = estop.reset();

    ASSERT_FALSE(result.has_value());
}

TEST(EmergencyStopController, WorksWithEmptyBrakeSpan) {
    auto controller = readyMachine();
    std::span<VirtualBrake> noBrakes;
    EmergencyStopController estop(controller, noBrakes);

    estop.trigger();

    EXPECT_EQ(controller.state(), ControllerState::EmergencyStop);
}
