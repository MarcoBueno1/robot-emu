// tests/controller/controller_state_machine_test.cpp
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
#include <initializer_list>
#include <vector>
#include "robot/controller/controller_state_machine.hpp"

using namespace robot::controller;

namespace {

// Applies a sequence of events, asserting each one succeeds. Used to walk
// a freshly constructed machine to some state of interest before the part
// of the test that actually matters.
//
// Two overloads on purpose:
//  - a template for any range (needed for std::vector<ControllerEvent>,
//    which is what GTest's TestWithParam<...> stores — see
//    EStopReachability below; a *stored* std::initializer_list would
//    dangle, since it never owns its backing array),
//  - an explicit std::initializer_list overload so call sites can still
//    pass a `{a, b, c}` brace list directly (braced-init-lists can't be
//    deduced by the template above — CTAD needs a concrete parameter type).
//    This one is safe: the list is consumed synchronously, within the
//    lifetime of the temporary backing array.
template <typename EventRange>
void driveTo(ControllerStateMachine& sm, const EventRange& events) {
    for (auto event : events) {
        auto result = sm.handleEvent(event);
        ASSERT_TRUE(result.has_value())
            << "Expected " << toString(event) << " to succeed from "
            << toString(sm.state());
    }
}

void driveTo(ControllerStateMachine& sm, std::initializer_list<ControllerEvent> events) {
    driveTo<std::initializer_list<ControllerEvent>>(sm, events);
}

constexpr std::array<ControllerEvent, 18> kAllEvents{
    ControllerEvent::PowerOn,          ControllerEvent::BootComplete,
    ControllerEvent::InitComplete,     ControllerEvent::ServoEnable,
    ControllerEvent::ServoDisable,     ControllerEvent::ControllerReady,
    ControllerEvent::CommandMove,      ControllerEvent::MotionComplete,
    ControllerEvent::CommandPause,     ControllerEvent::CommandResume,
    ControllerEvent::CommandStop,      ControllerEvent::StopComplete,
    ControllerEvent::EStopTriggered,   ControllerEvent::EStopReset,
    ControllerEvent::FaultDetected,    ControllerEvent::RecoveryStart,
    ControllerEvent::RecoveryComplete, ControllerEvent::PowerOff,
};

constexpr std::array<ControllerState, 14> kAllStates{
    ControllerState::PowerOff,      ControllerState::Booting,
    ControllerState::Initializing,  ControllerState::Idle,
    ControllerState::ServoOff,      ControllerState::ServoOn,
    ControllerState::Ready,         ControllerState::Moving,
    ControllerState::Paused,        ControllerState::Stopping,
    ControllerState::EmergencyStop, ControllerState::Fault,
    ControllerState::Recovery,      ControllerState::Shutdown,
};

}  // namespace

// --- Construction ---

TEST(ControllerStateMachine, StartsInPowerOff) {
    ControllerStateMachine sm;
    EXPECT_EQ(sm.state(), ControllerState::PowerOff);
}

// --- Boot chain (table rows 1-3) ---

TEST(ControllerStateMachine, BootChainReachesIdle) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete});
    EXPECT_EQ(sm.state(), ControllerState::Idle);
}

// --- Servo / Ready management (table rows 4-8) ---

TEST(ControllerStateMachine, ServoEnableFromIdleReachesServoOn) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable});
    EXPECT_EQ(sm.state(), ControllerState::ServoOn);
}

TEST(ControllerStateMachine, ServoEnableFromServoOffReachesServoOn) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ServoDisable, ControllerEvent::ServoEnable});
    EXPECT_EQ(sm.state(), ControllerState::ServoOn);
}

TEST(ControllerStateMachine, ServoDisableFromServoOnReachesServoOff) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ServoDisable});
    EXPECT_EQ(sm.state(), ControllerState::ServoOff);
}

TEST(ControllerStateMachine, ControllerReadyFromServoOnReachesReady) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ControllerReady});
    EXPECT_EQ(sm.state(), ControllerState::Ready);
}

TEST(ControllerStateMachine, ServoDisableFromReadyReachesServoOff) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ControllerReady, ControllerEvent::ServoDisable});
    EXPECT_EQ(sm.state(), ControllerState::ServoOff);
}

namespace {
// Shared setup for every test below that needs to start from Ready.
ControllerStateMachine readyMachine() {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ControllerReady});
    return sm;
}
}  // namespace

// --- Motion: move / pause / resume (table rows 9-12) ---

TEST(ControllerStateMachine, CommandMoveFromReadyReachesMoving) {
    auto sm = readyMachine();
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::CommandMove).has_value());
    EXPECT_EQ(sm.state(), ControllerState::Moving);
}

TEST(ControllerStateMachine, MotionCompleteFromMovingReachesReady) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandMove, ControllerEvent::MotionComplete});
    EXPECT_EQ(sm.state(), ControllerState::Ready);
}

TEST(ControllerStateMachine, CommandPauseFromMovingReachesPaused) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandMove, ControllerEvent::CommandPause});
    EXPECT_EQ(sm.state(), ControllerState::Paused);
}

TEST(ControllerStateMachine, CommandResumeFromPausedReachesMoving) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandMove, ControllerEvent::CommandPause, ControllerEvent::CommandResume});
    EXPECT_EQ(sm.state(), ControllerState::Moving);
}

// --- Stop (table rows 13-16) ---

TEST(ControllerStateMachine, CommandStopFromMovingReachesStopping) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandMove, ControllerEvent::CommandStop});
    EXPECT_EQ(sm.state(), ControllerState::Stopping);
}

TEST(ControllerStateMachine, CommandStopFromPausedReachesStopping) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandMove, ControllerEvent::CommandPause, ControllerEvent::CommandStop});
    EXPECT_EQ(sm.state(), ControllerState::Stopping);
}

TEST(ControllerStateMachine, CommandStopFromReadyReachesStopping) {
    auto sm = readyMachine();
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::CommandStop).has_value());
    EXPECT_EQ(sm.state(), ControllerState::Stopping);
}

TEST(ControllerStateMachine, StopCompleteFromStoppingReachesReady) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::CommandStop, ControllerEvent::StopComplete});
    EXPECT_EQ(sm.state(), ControllerState::Ready);
}

// --- Emergency stop (table rows 17-22) ---

class EStopReachability : public ::testing::TestWithParam<std::vector<ControllerEvent>> {};

TEST_P(EStopReachability, EStopTriggeredReachesEmergencyStop) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable});
    driveTo(sm, GetParam());
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::EStopTriggered).has_value());
    EXPECT_EQ(sm.state(), ControllerState::EmergencyStop);
}

INSTANTIATE_TEST_SUITE_P(
    FromEveryOperationalState, EStopReachability,
    ::testing::Values(
        std::vector<ControllerEvent>{},  // stays in ServoOn
        std::vector<ControllerEvent>{ControllerEvent::ControllerReady},  // -> Ready
        std::vector<ControllerEvent>{ControllerEvent::ControllerReady, ControllerEvent::CommandMove},  // -> Moving
        std::vector<ControllerEvent>{ControllerEvent::ControllerReady, ControllerEvent::CommandMove,
                                      ControllerEvent::CommandPause},  // -> Paused
        std::vector<ControllerEvent>{ControllerEvent::ControllerReady, ControllerEvent::CommandStop}));  // -> Stopping

TEST(ControllerStateMachine, EStopResetFromEmergencyStopReachesReady) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::EStopTriggered, ControllerEvent::EStopReset});
    EXPECT_EQ(sm.state(), ControllerState::Ready);
}

TEST(ControllerStateMachine, EmergencyStopOnlyAcceptsResetOrFault) {
    auto sm = readyMachine();
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::EStopTriggered).has_value());
    ASSERT_EQ(sm.state(), ControllerState::EmergencyStop);

    for (auto event : kAllEvents) {
        if (event == ControllerEvent::EStopReset || event == ControllerEvent::FaultDetected) {
            EXPECT_TRUE(sm.canHandle(event)) << toString(event) << " should be legal from EmergencyStop";
        } else {
            EXPECT_FALSE(sm.canHandle(event)) << toString(event) << " should NOT be legal from EmergencyStop";
        }
    }
}

// --- Fault / Recovery (table rows 23-32) ---

class FaultReachability : public ::testing::TestWithParam<ControllerState> {};

TEST_P(FaultReachability, FaultDetectedReachesFault) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete});

    switch (GetParam()) {
        case ControllerState::Idle:
            break;
        case ControllerState::ServoOff:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ServoDisable});
            break;
        case ControllerState::ServoOn:
            driveTo(sm, {ControllerEvent::ServoEnable});
            break;
        case ControllerState::Ready:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ControllerReady});
            break;
        case ControllerState::Moving:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ControllerReady, ControllerEvent::CommandMove});
            break;
        case ControllerState::Paused:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ControllerReady, ControllerEvent::CommandMove,
                         ControllerEvent::CommandPause});
            break;
        case ControllerState::Stopping:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ControllerReady, ControllerEvent::CommandStop});
            break;
        case ControllerState::EmergencyStop:
            driveTo(sm, {ControllerEvent::ServoEnable, ControllerEvent::ControllerReady, ControllerEvent::EStopTriggered});
            break;
        default:
            FAIL() << "Unhandled state in test setup: " << toString(GetParam());
    }

    ASSERT_EQ(sm.state(), GetParam());
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::FaultDetected).has_value());
    EXPECT_EQ(sm.state(), ControllerState::Fault);
}

INSTANTIATE_TEST_SUITE_P(
    FromEveryLiveState, FaultReachability,
    ::testing::Values(ControllerState::Idle, ControllerState::ServoOff, ControllerState::ServoOn,
                       ControllerState::Ready, ControllerState::Moving, ControllerState::Paused,
                       ControllerState::Stopping, ControllerState::EmergencyStop));

TEST(ControllerStateMachine, RecoveryStartFromFaultReachesRecovery) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::FaultDetected, ControllerEvent::RecoveryStart});
    EXPECT_EQ(sm.state(), ControllerState::Recovery);
}

TEST(ControllerStateMachine, RecoveryCompleteFromRecoveryReachesReady) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::FaultDetected, ControllerEvent::RecoveryStart, ControllerEvent::RecoveryComplete});
    EXPECT_EQ(sm.state(), ControllerState::Ready);
}

// --- Shutdown (table rows 33-36) ---

TEST(ControllerStateMachine, PowerOffFromIdleReachesShutdown) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::PowerOff});
    EXPECT_EQ(sm.state(), ControllerState::Shutdown);
}

TEST(ControllerStateMachine, PowerOffFromServoOffReachesShutdown) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::ServoEnable, ControllerEvent::ServoDisable, ControllerEvent::PowerOff});
    EXPECT_EQ(sm.state(), ControllerState::Shutdown);
}

TEST(ControllerStateMachine, PowerOffFromReadyReachesShutdown) {
    auto sm = readyMachine();
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::PowerOff).has_value());
    EXPECT_EQ(sm.state(), ControllerState::Shutdown);
}

TEST(ControllerStateMachine, PowerOffFromFaultReachesShutdown) {
    auto sm = readyMachine();
    driveTo(sm, {ControllerEvent::FaultDetected, ControllerEvent::PowerOff});
    EXPECT_EQ(sm.state(), ControllerState::Shutdown);
}

TEST(ControllerStateMachine, ShutdownAcceptsNoEvents) {
    ControllerStateMachine sm;
    driveTo(sm, {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                 ControllerEvent::PowerOff});
    ASSERT_EQ(sm.state(), ControllerState::Shutdown);

    for (auto event : kAllEvents) {
        EXPECT_FALSE(sm.canHandle(event)) << toString(event) << " should NOT be legal from Shutdown";
        auto result = sm.handleEvent(event);
        EXPECT_FALSE(result.has_value());
        EXPECT_EQ(sm.state(), ControllerState::Shutdown);
    }
}

// --- Invalid transitions leave state unchanged (acceptance criterion) ---

TEST(ControllerStateMachine, InvalidTransitionReturnsErrorAndLeavesStateUnchanged) {
    ControllerStateMachine sm;  // PowerOff

    auto result = sm.handleEvent(ControllerEvent::CommandMove);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ControllerError::InvalidTransition);
    EXPECT_EQ(sm.state(), ControllerState::PowerOff);
}

TEST(ControllerStateMachine, CanHandleMatchesHandleEventWithoutMutating) {
    ControllerStateMachine sm;

    EXPECT_TRUE(sm.canHandle(ControllerEvent::PowerOn));
    EXPECT_FALSE(sm.canHandle(ControllerEvent::CommandMove));
    EXPECT_EQ(sm.state(), ControllerState::PowerOff)
        << "canHandle() must not mutate state";
}

// --- toString coverage ---

TEST(ControllerStateMachine, ToStringCoversEveryState) {
    for (auto state : kAllStates) {
        EXPECT_FALSE(toString(state).empty()) << "No toString() text for a ControllerState";
    }
    EXPECT_EQ(toString(ControllerState::Ready), "Ready");
    EXPECT_EQ(toString(ControllerState::EmergencyStop), "EmergencyStop");
}

TEST(ControllerStateMachine, ToStringCoversEveryEvent) {
    for (auto event : kAllEvents) {
        EXPECT_FALSE(toString(event).empty()) << "No toString() text for a ControllerEvent";
    }
    EXPECT_EQ(toString(ControllerEvent::CommandMove), "CommandMove");
    EXPECT_EQ(toString(ControllerEvent::FaultDetected), "FaultDetected");
}
