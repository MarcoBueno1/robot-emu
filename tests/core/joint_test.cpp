// tests/core/joint_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/core/joint.hpp"

using namespace robot::core;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0,
        .max_position     =  180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

}  // namespace

TEST(Joint, StartsDisabledAndNotHomed) {
    Joint joint(defaultLimits());
    EXPECT_FALSE(joint.enabled());
    EXPECT_FALSE(joint.homed());
}

TEST(Joint, RejectsTargetPositionBeyondLimits) {
    Joint joint(defaultLimits());
    joint.enable();

    auto result = joint.setTargetPosition(500.0);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::PositionOutOfRange);
}

TEST(Joint, DoesNotMoveWhileDisabled) {
    Joint joint(defaultLimits());
    // joint stays disabled on purpose

    auto set = joint.setTargetPosition(90.0);
    ASSERT_FALSE(set.has_value());
    EXPECT_EQ(set.error(), JointError::JointDisabled);

    joint.update(std::chrono::milliseconds(10));
    EXPECT_DOUBLE_EQ(joint.position(), 0.0);
}

TEST(Joint, DoesNotMoveAfterBeingDisabledEvenWithPriorTarget) {
    Joint joint(defaultLimits());
    joint.enable();
    ASSERT_TRUE(joint.setTargetPosition(45.0).has_value());

    joint.disable();
    for (int i = 0; i < 100; ++i) {
        joint.update(std::chrono::milliseconds(1));
    }

    EXPECT_DOUBLE_EQ(joint.position(), 0.0);
    EXPECT_DOUBLE_EQ(joint.velocity(), 0.0);
}

TEST(Joint, ConvergesTowardTargetPositionRespectingVelocityLimit) {
    Joint joint(defaultLimits());
    joint.enable();
    // NOTE: target chosen to be reachable within the simulated window given
    // this joint's max_velocity (2.0 rad/s) — 1.5 rad is reachable well
    // within 1 simulated second, unlike a larger target such as 10.0 rad,
    // which would require an average speed above max_velocity.
    ASSERT_TRUE(joint.setTargetPosition(1.5).has_value());

    // 1 simulated second in 1 ms steps — deterministic, no real sleep.
    for (int i = 0; i < 1000; ++i) {
        joint.update(std::chrono::milliseconds(1));
    }

    EXPECT_LE(joint.velocity(), defaultLimits().max_velocity);
    EXPECT_NEAR(joint.position(), 1.5, 0.5);
}

TEST(Joint, HomeResetsPositionAndMarksHomed) {
    Joint joint(defaultLimits());
    joint.enable();
    ASSERT_TRUE(joint.setTargetPosition(45.0).has_value());
    for (int i = 0; i < 500; ++i) joint.update(std::chrono::milliseconds(1));

    joint.home();

    EXPECT_TRUE(joint.homed());
    EXPECT_DOUBLE_EQ(joint.position(), 0.0);
}
