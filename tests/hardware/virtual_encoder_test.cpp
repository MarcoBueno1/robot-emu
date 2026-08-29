// tests/hardware/virtual_encoder_test.cpp
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
#include "robot/hardware/virtual_encoder.hpp"

using namespace robot::hardware;

TEST(VirtualEncoder, RejectsZeroCountsPerRevolution) {
    auto result = VirtualEncoder::create(0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), HardwareError::InvalidConfiguration);
}

TEST(VirtualEncoder, ResolutionMatchesTwoPiOverCounts) {
    auto encoder = VirtualEncoder::create(4).value();
    EXPECT_NEAR(encoder.resolutionRadians(), std::numbers::pi / 2.0, 1e-12);
}

TEST(VirtualEncoder, CountsRoundsToNearestStep) {
    // counts_per_revolution = 4 -> resolution = pi/2 ~= 1.5707963267948966
    auto encoder = VirtualEncoder::create(4).value();

    EXPECT_EQ(encoder.counts(0.0), 0);
    EXPECT_EQ(encoder.counts(0.1), 0);    // 0.1 / (pi/2) ~= 0.0637 -> rounds to 0
    EXPECT_EQ(encoder.counts(1.0), 1);    // 1.0 / (pi/2) ~= 0.6366 -> rounds to 1
    EXPECT_EQ(encoder.counts(2.5), 2);    // 2.5 / (pi/2) ~= 1.5915 -> rounds to 2
}

TEST(VirtualEncoder, CountsRoundsNegativeValuesAwayFromZero) {
    auto encoder = VirtualEncoder::create(4).value();

    EXPECT_EQ(encoder.counts(-1.0), -1);  // -1.0 / (pi/2) ~= -0.6366 -> rounds to -1
    EXPECT_EQ(encoder.counts(-0.1), 0);
}

TEST(VirtualEncoder, SampleIsCountsTimesResolution) {
    auto encoder = VirtualEncoder::create(4).value();

    EXPECT_NEAR(encoder.sample(1.0), 1.0 * encoder.resolutionRadians(), 1e-12);
    EXPECT_NEAR(encoder.sample(-1.0), -1.0 * encoder.resolutionRadians(), 1e-12);
    EXPECT_NEAR(encoder.sample(2.5), 2.0 * encoder.resolutionRadians(), 1e-12);
}

TEST(VirtualEncoder, HigherResolutionEncoderQuantizesMoreFinely) {
    auto coarse = VirtualEncoder::create(4).value();
    auto fine = VirtualEncoder::create(4096).value();

    EXPECT_GT(coarse.resolutionRadians(), fine.resolutionRadians());

    // A small true position that a coarse encoder rounds down to zero
    // should still be resolvable by a much finer one.
    constexpr double smallAngle = 0.01;
    EXPECT_EQ(coarse.counts(smallAngle), 0);
    EXPECT_NE(fine.counts(smallAngle), 0);
}
