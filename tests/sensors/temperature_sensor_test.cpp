// tests/sensors/temperature_sensor_test.cpp
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
#include "robot/sensors/temperature_sensor.hpp"

using namespace robot::sensors;
using robot::hardware::HardwareError;

TEST(TemperatureSensor, RejectsNonPositiveTimeConstant) {
    auto result = TemperatureSensor::create(25.0, std::chrono::nanoseconds(0), 80.0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(TemperatureSensor, StartsAtAmbient) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();
    EXPECT_DOUBLE_EQ(sensor.celsius(), 25.0);
}

TEST(TemperatureSensor, ReachesApproximatelyOneMinusOneOverEAfterOneTimeConstant) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();

    sensor.update(std::chrono::milliseconds(100), 75.0);  // exactly one tau, target 75 (range 50)

    const double expected = 25.0 + 50.0 * (1.0 - std::exp(-1.0));  // ~56.6
    EXPECT_NEAR(sensor.celsius(), expected, 1e-9);
}

TEST(TemperatureSensor, ConvergesToTargetAfterManyTimeConstants) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();

    for (int i = 0; i < 50; ++i) {
        sensor.update(std::chrono::milliseconds(100), 75.0);
    }

    EXPECT_NEAR(sensor.celsius(), 75.0, 1e-6);
}

TEST(TemperatureSensor, IsOverTemperatureFalseBelowThreshold) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();
    EXPECT_FALSE(sensor.isOverTemperature());
}

TEST(TemperatureSensor, IsOverTemperatureTrueAboveThreshold) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();
    for (int i = 0; i < 100; ++i) {
        sensor.update(std::chrono::milliseconds(100), 200.0);
    }
    EXPECT_TRUE(sensor.isOverTemperature());
}

TEST(TemperatureSensor, ZeroDtLeavesTemperatureUnchanged) {
    auto sensor = TemperatureSensor::create(25.0, std::chrono::milliseconds(100), 80.0).value();
    sensor.update(std::chrono::nanoseconds(0), 100.0);
    EXPECT_DOUBLE_EQ(sensor.celsius(), 25.0);
}
