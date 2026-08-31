// tests/sensors/encoder_sensor_test.cpp
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
#include <numeric>
#include "robot/sensors/encoder_sensor.hpp"

using namespace robot::sensors;
using robot::hardware::HardwareError;
using robot::hardware::VirtualEncoder;

namespace {
VirtualEncoder defaultEncoder() {
    return VirtualEncoder::create(4096).value();
}
}  // namespace

TEST(EncoderSensor, RejectsNegativeNoiseStdDev) {
    auto result = EncoderSensor::create(defaultEncoder(), -0.1, 0.0, 42);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(EncoderSensor, NoNoiseMatchesRawVirtualEncoderSample) {
    auto encoder = defaultEncoder();
    auto sensor = EncoderSensor::create(encoder, 0.0, 0.0, 1).value();

    for (double truePos : {0.0, 0.5, -1.2, 3.0}) {
        EXPECT_DOUBLE_EQ(sensor.read(truePos), encoder.sample(truePos));
    }
}

TEST(EncoderSensor, OffsetShiftsPositionBeforeQuantization) {
    auto encoder = defaultEncoder();
    auto sensor = EncoderSensor::create(encoder, 0.0, 0.5, 1).value();

    EXPECT_DOUBLE_EQ(sensor.read(0.0), encoder.sample(0.5));
}

TEST(EncoderSensor, SameSeedProducesIdenticalSequence) {
    auto sensor1 = EncoderSensor::create(defaultEncoder(), 0.01, 0.0, 777).value();
    auto sensor2 = EncoderSensor::create(defaultEncoder(), 0.01, 0.0, 777).value();

    for (int i = 0; i < 20; ++i) {
        double truePos = static_cast<double>(i) * 0.1;
        EXPECT_DOUBLE_EQ(sensor1.read(truePos), sensor2.read(truePos));
    }
}

TEST(EncoderSensor, NoiseStdDevMatchesConfiguredValueWithinTolerance) {
    auto encoder = defaultEncoder();
    const double configuredStdDev = 0.02;
    auto sensor = EncoderSensor::create(encoder, configuredStdDev, 0.0, 12345).value();

    constexpr int kSamples = 2000;
    std::vector<double> noiseComponents;
    noiseComponents.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        constexpr double truePos = 1.0;  // fixed true position; only noise varies
        double reading = sensor.read(truePos);
        double noiseless = encoder.sample(truePos);
        noiseComponents.push_back(reading - noiseless);
    }

    double mean = std::accumulate(noiseComponents.begin(), noiseComponents.end(), 0.0) / kSamples;
    double variance = 0.0;
    for (double v : noiseComponents) {
        variance += (v - mean) * (v - mean);
    }
    variance /= kSamples;
    double stdDev = std::sqrt(variance);

    // Generous tolerance — statistical test, not an exact-value assertion.
    EXPECT_NEAR(stdDev, configuredStdDev, configuredStdDev * 0.2);
}

TEST(EncoderSensor, StuckFreezesLastReadingRegardlessOfNewInput) {
    auto sensor = EncoderSensor::create(defaultEncoder(), 0.0, 0.0, 1).value();

    double frozenValue = sensor.read(1.0);
    sensor.setFailureMode(EncoderFailureMode::Stuck);

    EXPECT_DOUBLE_EQ(sensor.read(5.0), frozenValue);
    EXPECT_DOUBLE_EQ(sensor.read(-3.0), frozenValue);
    EXPECT_DOUBLE_EQ(sensor.read(100.0), frozenValue);
}

TEST(EncoderSensor, DisconnectedReturnsNaN) {
    auto sensor = EncoderSensor::create(defaultEncoder(), 0.0, 0.0, 1).value();
    sensor.setFailureMode(EncoderFailureMode::Disconnected);

    EXPECT_TRUE(std::isnan(sensor.read(0.0)));
    EXPECT_TRUE(std::isnan(sensor.read(99.0)));
}

TEST(EncoderSensor, FailureModeStartsAsNone) {
    auto sensor = EncoderSensor::create(defaultEncoder(), 0.0, 0.0, 1).value();
    EXPECT_EQ(sensor.failureMode(), EncoderFailureMode::None);
}

TEST(EncoderSensor, ReturningToNoneResumesNormalReadings) {
    auto encoder = defaultEncoder();
    auto sensor = EncoderSensor::create(encoder, 0.0, 0.0, 1).value();

    sensor.setFailureMode(EncoderFailureMode::Disconnected);
    [[maybe_unused]] auto discarded = sensor.read(0.0);
    sensor.setFailureMode(EncoderFailureMode::None);

    EXPECT_DOUBLE_EQ(sensor.read(2.0), encoder.sample(2.0));
}
