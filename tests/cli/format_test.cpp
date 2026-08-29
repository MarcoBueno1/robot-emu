// tests/cli/format_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <numbers>
#include "robot/cli/format.hpp"

using namespace robot::cli;

TEST(FormatStatusTable, ContainsRobotNameAndState) {
    StatusPayload status{.robotName = "RE-6AXIS", .controllerStateName = "Ready", .joints = {}};

    auto text = formatStatusTable(status);

    EXPECT_NE(text.find("RE-6AXIS"), std::string::npos);
    EXPECT_NE(text.find("Ready"), std::string::npos);
}

TEST(FormatStatusTable, RendersJointPositionInDegrees) {
    StatusPayload status{.robotName = "Arm",
                          .controllerStateName = "Ready",
                          .joints = {{std::numbers::pi / 2.0, 0.0}}};  // 90 degrees

    auto text = formatStatusTable(status);

    EXPECT_NE(text.find("90.00"), std::string::npos) << text;
}

TEST(FormatStatusTable, RendersOneLinePerJoint) {
    StatusPayload status{.robotName = "Arm", .controllerStateName = "Ready", .joints = {{0.0, 0.0}, {0.0, 0.0}}};

    auto text = formatStatusTable(status);

    EXPECT_NE(text.find("J1"), std::string::npos);
    EXPECT_NE(text.find("J2"), std::string::npos);
}

TEST(FormatStatusTable, HandlesNoJointsGracefully) {
    StatusPayload status{.robotName = "Arm", .controllerStateName = "Idle", .joints = {}};

    auto text = formatStatusTable(status);

    EXPECT_NE(text.find("Arm"), std::string::npos);
    EXPECT_NE(text.find("Idle"), std::string::npos);
}
