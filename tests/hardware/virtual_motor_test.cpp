// tests/hardware/virtual_motor_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include <cmath>
#include "robot/hardware/virtual_motor.hpp"

using namespace robot::hardware;

TEST(VirtualMotor, RejectsNonPositiveMaxTorque) {
    auto result = VirtualMotor::create(0.0, std::chrono::milliseconds(100));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(VirtualMotor, RejectsNonPositiveTimeConstant) {
    auto result = VirtualMotor::create(10.0, std::chrono::nanoseconds(0));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(VirtualMotor, StartsAtZeroTorque) {
    auto motor = VirtualMotor::create(10.0, std::chrono::milliseconds(100)).value();
    EXPECT_DOUBLE_EQ(motor.torque(), 0.0);
    EXPECT_DOUBLE_EQ(motor.commandedTorque(), 0.0);
}

TEST(VirtualMotor, RejectsCommandedTorqueBeyondLimit) {
    auto motor = VirtualMotor::create(10.0, std::chrono::milliseconds(100)).value();

    auto result = motor.setCommandedTorque(20.0);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::OutOfRange);
    EXPECT_DOUBLE_EQ(motor.commandedTorque(), 0.0);
}

TEST(VirtualMotor, ReachesApproximatelyOneMinusOneOverEAfterOneTimeConstant) {
    auto motor = VirtualMotor::create(10.0, std::chrono::milliseconds(100)).value();
    ASSERT_TRUE(motor.setCommandedTorque(10.0).has_value());

    motor.update(std::chrono::milliseconds(100));  // exactly one tau

    const double expected = 10.0 * (1.0 - std::exp(-1.0));  // ~6.32
    EXPECT_NEAR(motor.torque(), expected, 1e-9);
}

TEST(VirtualMotor, ConvergesToCommandedTorqueAfterManyTimeConstants) {
    auto motor = VirtualMotor::create(10.0, std::chrono::milliseconds(100)).value();
    ASSERT_TRUE(motor.setCommandedTorque(10.0).has_value());

    for (int i = 0; i < 50; ++i) {
        motor.update(std::chrono::milliseconds(100));
    }

    EXPECT_NEAR(motor.torque(), 10.0, 1e-6);
}

TEST(VirtualMotor, ZeroDtLeavesTorqueUnchanged) {
    auto motor = VirtualMotor::create(10.0, std::chrono::milliseconds(100)).value();
    ASSERT_TRUE(motor.setCommandedTorque(10.0).has_value());

    motor.update(std::chrono::nanoseconds(0));

    EXPECT_DOUBLE_EQ(motor.torque(), 0.0);
}
