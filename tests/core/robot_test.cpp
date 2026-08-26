// tests/core/robot_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/core/robot.hpp"

using namespace robot::core;

TEST(Robot, CreateFailsWithZeroJoints) {
    RobotConfig config{.name = "empty", .joints = {}};

    auto result = Robot::create(config);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::InvalidConfiguration);
}

TEST(Robot, CreateFailsWithInconsistentLimits) {
    RobotConfig config{
        .name = "bad",
        .joints = {JointLimits{
            .min_position = 10.0, .max_position = -10.0,  // min > max: invalid
            .max_velocity = 2.0,  .max_acceleration = 5.0,
        }},
    };

    auto result = Robot::create(config);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::InvalidConfiguration);
}

TEST(Robot, CreateSucceedsWithValidSixAxisConfig) {
    RobotConfig config{
        .name = "RE-6AXIS",
        .joints = std::vector<JointLimits>(6, JointLimits{
            .min_position = -180.0, .max_position = 180.0,
            .max_velocity = 2.0,    .max_acceleration = 5.0,
        }),
    };

    auto result = Robot::create(config);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->jointCount(), 6u);
}

TEST(Robot, UpdateAdvancesAllJointsInOrder) {
    RobotConfig config{
        .name = "RE-6AXIS",
        .joints = std::vector<JointLimits>(6, JointLimits{
            .min_position = -180.0, .max_position = 180.0,
            .max_velocity = 2.0,    .max_acceleration = 5.0,
        }),
    };
    auto robot = Robot::create(config).value();

    for (auto& j : robot.joints()) {
        j.enable();
        // NOTE: target chosen to be reachable within the simulated window
        // (0.5 s) given max_velocity = 2.0 rad/s.
        ASSERT_TRUE(j.setTargetPosition(0.8).has_value());
    }

    for (int i = 0; i < 500; ++i) {
        robot.update(std::chrono::milliseconds(1));
    }

    for (const auto& j : robot.joints()) {
        EXPECT_NEAR(j.position(), 0.8, 1.0);
    }
}
