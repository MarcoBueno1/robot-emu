// tests/cli/payload_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/cli/move_joint_payload.hpp"
#include "robot/cli/status_payload.hpp"

using namespace robot::cli;

// --- StatusPayload ---

TEST(StatusPayload, RoundTripsWithNoJoints) {
    StatusPayload status{.robotName = "RE-6AXIS", .controllerStateName = "Ready", .joints = {}};

    auto encoded = encodeStatusPayload(status);
    auto decoded = decodeStatusPayload(encoded);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->robotName, "RE-6AXIS");
    EXPECT_EQ(decoded->controllerStateName, "Ready");
    EXPECT_TRUE(decoded->joints.empty());
}

TEST(StatusPayload, RoundTripsWithSeveralJoints) {
    StatusPayload status{.robotName = "Arm", .controllerStateName = "Moving",
                          .joints = {{0.1, 0.2}, {-1.5, 0.0}, {3.14159, -0.5}}};

    auto encoded = encodeStatusPayload(status);
    auto decoded = decodeStatusPayload(encoded);

    ASSERT_TRUE(decoded.has_value());
    ASSERT_EQ(decoded->joints.size(), 3u);
    EXPECT_DOUBLE_EQ(decoded->joints[0].positionRadians, 0.1);
    EXPECT_DOUBLE_EQ(decoded->joints[0].velocityRadiansPerSecond, 0.2);
    EXPECT_DOUBLE_EQ(decoded->joints[1].positionRadians, -1.5);
    EXPECT_DOUBLE_EQ(decoded->joints[2].positionRadians, 3.14159);
    EXPECT_DOUBLE_EQ(decoded->joints[2].velocityRadiansPerSecond, -0.5);
}

TEST(StatusPayload, RejectsTruncatedBuffer) {
    StatusPayload status{.robotName = "RE-6AXIS", .controllerStateName = "Ready", .joints = {{1.0, 2.0}}};
    auto encoded = encodeStatusPayload(status);

    std::span<const std::byte> truncated(encoded.data(), encoded.size() - 3);
    auto result = decodeStatusPayload(truncated);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::TruncatedPayload);
}

TEST(StatusPayload, RejectsEmptyBuffer) {
    auto result = decodeStatusPayload(std::span<const std::byte>{});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::TruncatedPayload);
}

// --- MoveJointPayload ---

TEST(MoveJointPayload, RoundTrips) {
    MoveJointPayload payload{.jointIndex = 4, .targetRadians = 1.5707963267948966};

    auto encoded = encodeMoveJointPayload(payload);
    auto decoded = decodeMoveJointPayload(encoded);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->jointIndex, 4);
    EXPECT_DOUBLE_EQ(decoded->targetRadians, 1.5707963267948966);
}

TEST(MoveJointPayload, RoundTripsNegativeTarget) {
    MoveJointPayload payload{.jointIndex = 0, .targetRadians = -3.14159};

    auto encoded = encodeMoveJointPayload(payload);
    auto decoded = decodeMoveJointPayload(encoded);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_DOUBLE_EQ(decoded->targetRadians, -3.14159);
}

TEST(MoveJointPayload, RejectsTruncatedBuffer) {
    MoveJointPayload payload{.jointIndex = 1, .targetRadians = 1.0};
    auto encoded = encodeMoveJointPayload(payload);

    std::span<const std::byte> truncated(encoded.data(), encoded.size() - 1);
    auto result = decodeMoveJointPayload(truncated);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::TruncatedPayload);
}

TEST(MoveJointPayload, RejectsEmptyBuffer) {
    auto result = decodeMoveJointPayload(std::span<const std::byte>{});
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), CliError::TruncatedPayload);
}
