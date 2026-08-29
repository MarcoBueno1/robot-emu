// tests/hardware/virtual_brake_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/hardware/virtual_brake.hpp"

using namespace robot::hardware;

TEST(VirtualBrake, StartsEngaged) {
    VirtualBrake brake;
    EXPECT_EQ(brake.state(), BrakeState::Engaged);
    EXPECT_TRUE(brake.isEngaged());
}

TEST(VirtualBrake, ReleaseChangesState) {
    VirtualBrake brake;
    brake.release();
    EXPECT_EQ(brake.state(), BrakeState::Released);
    EXPECT_FALSE(brake.isEngaged());
}

TEST(VirtualBrake, EngageAfterReleaseReturnsToEngaged) {
    VirtualBrake brake;
    brake.release();
    brake.engage();
    EXPECT_TRUE(brake.isEngaged());
}

TEST(VirtualBrake, ReleaseIsIdempotent) {
    VirtualBrake brake;
    brake.release();
    brake.release();
    EXPECT_EQ(brake.state(), BrakeState::Released);
}

TEST(VirtualBrake, EngageIsIdempotent) {
    VirtualBrake brake;
    brake.engage();
    brake.engage();
    EXPECT_EQ(brake.state(), BrakeState::Engaged);
}
