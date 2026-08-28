// tests/runtime/control_loop_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "robot/controller/controller_state_machine.hpp"
#include "robot/core/robot.hpp"
#include "robot/runtime/control_loop.hpp"

using namespace robot::runtime;
using robot::controller::ControllerEvent;
using robot::controller::ControllerState;
using robot::controller::ControllerStateMachine;
using robot::core::Joint;
using robot::core::JointLimits;
using robot::core::Robot;
using robot::core::RobotConfig;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0,
        .max_position     =  180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

Robot singleJointRobot() {
    RobotConfig config{.name = "test", .joints = {defaultLimits()}};
    return Robot::create(config).value();
}

// Drives sm through the boot chain and servo-enable so it lands in Ready,
// asserting every step succeeds — same pattern used in Phase 2's own tests.
void driveToReady(ControllerStateMachine& sm) {
    for (auto event : {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                        ControllerEvent::ServoEnable, ControllerEvent::ControllerReady}) {
        ASSERT_TRUE(sm.handleEvent(event).has_value());
    }
    ASSERT_EQ(sm.state(), ControllerState::Ready);
}

}  // namespace

// --- period() ---

TEST(ControlLoop, PeriodMatchesEveryFrequencyExactly) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;

    EXPECT_EQ(ControlLoop(robot, sm, ControlLoopFrequency::Hz500).period(), std::chrono::nanoseconds(2'000'000));
    EXPECT_EQ(ControlLoop(robot, sm, ControlLoopFrequency::Hz1000).period(), std::chrono::nanoseconds(1'000'000));
    EXPECT_EQ(ControlLoop(robot, sm, ControlLoopFrequency::Hz2000).period(), std::chrono::nanoseconds(500'000));
    EXPECT_EQ(ControlLoop(robot, sm, ControlLoopFrequency::Hz5000).period(), std::chrono::nanoseconds(200'000));
    EXPECT_EQ(ControlLoop(robot, sm, ControlLoopFrequency::Hz10000).period(), std::chrono::nanoseconds(100'000));
}

// --- step(): deterministic, synchronous, no thread ---

TEST(ControlLoop, StepDoesNotMoveRobotWhileNotMoving) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;  // stays PowerOff
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    for (int i = 0; i < 50; ++i) {
        loop.step();
    }

    EXPECT_DOUBLE_EQ(robot.joint(0).position(), 0.0);
    EXPECT_EQ(loop.metrics().cyclesExecuted, 50u);
}

TEST(ControlLoop, StepDoesNotMoveRobotWhileReadyButNotMoving) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    driveToReady(sm);
    robot.joint(0).enable();
    ASSERT_TRUE(robot.joint(0).setTargetPosition(1.5).has_value());

    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);
    for (int i = 0; i < 50; ++i) {
        loop.step();
    }

    EXPECT_DOUBLE_EQ(robot.joint(0).position(), 0.0)
        << "Robot must not move while the controller is Ready but not yet Moving";
}

TEST(ControlLoop, StepMovesRobotWhileMoving) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    driveToReady(sm);
    robot.joint(0).enable();
    ASSERT_TRUE(robot.joint(0).setTargetPosition(1.5).has_value());
    ASSERT_TRUE(sm.handleEvent(ControllerEvent::CommandMove).has_value());
    ASSERT_EQ(sm.state(), ControllerState::Moving);

    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);
    for (int i = 0; i < 1000; ++i) {
        loop.step();
    }

    EXPECT_NEAR(robot.joint(0).position(), 1.5, 0.5);
    EXPECT_EQ(loop.metrics().cyclesExecuted, 1000u);
}

TEST(ControlLoop, StepIncrementsCyclesExecutedRegardlessOfState) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;  // PowerOff the whole time
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz500);

    ASSERT_EQ(loop.metrics().cyclesExecuted, 0u);
    loop.step();
    EXPECT_EQ(loop.metrics().cyclesExecuted, 1u);
    loop.step();
    EXPECT_EQ(loop.metrics().cyclesExecuted, 2u);
}

TEST(ControlLoop, DirectStepCallsLeaveAverageCycleTimeAtZero) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    for (int i = 0; i < 10; ++i) {
        loop.step();
    }

    EXPECT_EQ(loop.metrics().averageCycleTime, std::chrono::nanoseconds(0));
    EXPECT_EQ(loop.metrics().deadlineMisses, 0u);
}

// --- start()/stop() lifecycle errors ---

TEST(ControlLoop, StartFailsWhenAlreadyRunning) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    ASSERT_TRUE(loop.start().has_value());
    auto second = loop.start();
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), ControlLoopError::AlreadyRunning);

    ASSERT_TRUE(loop.stop().has_value());
}

TEST(ControlLoop, StopFailsWhenNotRunning) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    auto result = loop.stop();
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), ControlLoopError::NotRunning);
}

TEST(ControlLoop, IsRunningReflectsLifecycle) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    EXPECT_FALSE(loop.isRunning());
    ASSERT_TRUE(loop.start().has_value());
    EXPECT_TRUE(loop.isRunning());
    ASSERT_TRUE(loop.stop().has_value());
    EXPECT_FALSE(loop.isRunning());
}

// --- Background thread smoke tests (real timing — loose, generous bounds
//     by design; see docs/task-briefs/phase-03-control-loop.md section 8) ---

TEST(ControlLoop, StartAdvancesCyclesAndStopHaltsIt) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);  // 1ms period

    ASSERT_TRUE(loop.start().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ASSERT_TRUE(loop.stop().has_value());

    const auto cyclesAfterStop = loop.metrics().cyclesExecuted;
    EXPECT_GT(cyclesAfterStop, 0u) << "Expected at least some cycles in 30ms at 1kHz";

    // Give any (unexpected) still-running work a chance to run, then
    // confirm the count really did stop moving.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(loop.metrics().cyclesExecuted, cyclesAfterStop);
}

TEST(ControlLoop, DestructorStopsStillRunningThread) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;

    {
        ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);
        ASSERT_TRUE(loop.start().has_value());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // loop destructs here without an explicit stop() call — under
        // AddressSanitizer/UBSan (enabled in the Debug build this test
        // suite runs under), any lifetime or data-race bug here would be
        // caught, not just a crash in an ordinary build.
    }

    SUCCEED();
}

TEST(ControlLoop, ThreadedRunPopulatesAverageCycleTime) {
    auto robot = singleJointRobot();
    ControllerStateMachine sm;
    ControlLoop loop(robot, sm, ControlLoopFrequency::Hz1000);

    ASSERT_TRUE(loop.start().has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ASSERT_TRUE(loop.stop().has_value());

    // A real cycle takes at least some nonzero time; loose upper bound
    // (10ms) is far above what a ~1ms-period cycle should ever take, even
    // under heavy CI/sandbox scheduling noise — this just guards against
    // the field being wildly wrong, not against ordinary jitter.
    EXPECT_GT(loop.metrics().averageCycleTime.count(), 0);
    EXPECT_LT(loop.metrics().averageCycleTime, std::chrono::milliseconds(10));
}
