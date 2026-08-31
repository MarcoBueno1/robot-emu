// tests/sensors/current_sensor_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/sensors/current_sensor.hpp"

using namespace robot::sensors;
using robot::hardware::HardwareError;

TEST(CurrentSensor, RejectsNonPositiveMaxSafeAmps) {
    auto result = CurrentSensor::create(0.0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(CurrentSensor, NotOverCurrentBelowThreshold) {
    auto sensor = CurrentSensor::create(10.0).value();
    EXPECT_FALSE(sensor.isOverCurrent(5.0));
}

TEST(CurrentSensor, NotOverCurrentExactlyAtThreshold) {
    auto sensor = CurrentSensor::create(10.0).value();
    EXPECT_FALSE(sensor.isOverCurrent(10.0));
}

TEST(CurrentSensor, OverCurrentAboveThreshold) {
    auto sensor = CurrentSensor::create(10.0).value();
    EXPECT_TRUE(sensor.isOverCurrent(10.1));
}

TEST(CurrentSensor, UsesMagnitudeForNegativeCurrent) {
    auto sensor = CurrentSensor::create(10.0).value();
    EXPECT_FALSE(sensor.isOverCurrent(-5.0));
    EXPECT_TRUE(sensor.isOverCurrent(-10.1));
}
