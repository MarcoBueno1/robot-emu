// tests/safety/limit_monitor_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/safety/limit_monitor.hpp"

using namespace robot::safety;
using robot::core::JointLimits;
using robot::core::Robot;
using robot::core::RobotConfig;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -10.0,
        .max_position     =  10.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

Robot singleJointRobotAt(double startPosition) {
    RobotConfig config{.name = "test", .joints = {defaultLimits()}};
    auto robot = Robot::create(config).value();
    robot.joint(0).enable();
    // Drive the joint to startPosition deterministically (no real sleep —
    // same pattern as Phase 1/3's own tests). 10000 x 1ms = 10 simulated
    // seconds, enough for this controller to fully converge even across
    // this joint's full ±10 rad range at max_velocity = 2 rad/s.
    if (robot.joint(0).setTargetPosition(startPosition).has_value()) {
        for (int i = 0; i < 10000; ++i) {
            robot.joint(0).update(std::chrono::milliseconds(1));
        }
    }
    return robot;
}

}  // namespace

TEST(LimitMonitor, NoViolationsWellWithinLimits) {
    auto robot = singleJointRobotAt(0.0);
    LimitMonitor monitor(0.5);

    auto violations = monitor.check(robot);

    EXPECT_TRUE(violations.empty());
}

TEST(LimitMonitor, FlagsJointNearMaxPosition) {
    auto robot = singleJointRobotAt(9.8);  // within 0.5 of max_position (10.0)
    LimitMonitor monitor(0.5);

    auto violations = monitor.check(robot);

    ASSERT_EQ(violations.size(), 1u);
    EXPECT_EQ(violations[0].jointIndex, 0u);
    EXPECT_NEAR(violations[0].position, 9.8, 1e-6);
    EXPECT_NEAR(violations[0].distanceToNearestLimit, 0.2, 1e-6);
}

TEST(LimitMonitor, FlagsJointNearMinPosition) {
    auto robot = singleJointRobotAt(-9.7);  // within 0.5 of min_position (-10.0)
    LimitMonitor monitor(0.5);

    auto violations = monitor.check(robot);

    ASSERT_EQ(violations.size(), 1u);
    EXPECT_NEAR(violations[0].distanceToNearestLimit, 0.3, 1e-6);
}

TEST(LimitMonitor, DistanceToNearestLimitIsNeverNegative) {
    auto robot = singleJointRobotAt(10.0);  // exactly at max_position
    LimitMonitor monitor(1.0);

    auto violations = monitor.check(robot);

    ASSERT_EQ(violations.size(), 1u);
    EXPECT_GE(violations[0].distanceToNearestLimit, 0.0);
}

TEST(LimitMonitor, ChecksEveryJointInOrder) {
    RobotConfig config{.name = "multi", .joints = {defaultLimits(), defaultLimits(), defaultLimits()}};
    auto robot = Robot::create(config).value();
    for (auto& joint : robot.joints()) {
        joint.enable();
    }
    ASSERT_TRUE(robot.joint(1).setTargetPosition(9.9).has_value());
    for (int i = 0; i < 10000; ++i) {
        robot.update(std::chrono::milliseconds(1));
    }

    LimitMonitor monitor(0.5);
    auto violations = monitor.check(robot);

    ASSERT_EQ(violations.size(), 1u);
    EXPECT_EQ(violations[0].jointIndex, 1u);
}
