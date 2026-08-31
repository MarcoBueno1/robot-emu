// tests/sensors/proximity_sensor_test.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include <gtest/gtest.h>
#include "robot/sensors/proximity_sensor.hpp"

using namespace robot::sensors;
using robot::hardware::HardwareError;

TEST(ProximitySensor, RejectsNonPositiveTriggerDistance) {
    auto result = ProximitySensor::create(0.0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(ProximitySensor, NotTriggeredBeyondDistance) {
    auto sensor = ProximitySensor::create(0.1).value();
    EXPECT_FALSE(sensor.isTriggered(0.5));
}

TEST(ProximitySensor, TriggeredExactlyAtDistance) {
    auto sensor = ProximitySensor::create(0.1).value();
    EXPECT_TRUE(sensor.isTriggered(0.1));
}

TEST(ProximitySensor, TriggeredCloserThanDistance) {
    auto sensor = ProximitySensor::create(0.1).value();
    EXPECT_TRUE(sensor.isTriggered(0.01));
}
