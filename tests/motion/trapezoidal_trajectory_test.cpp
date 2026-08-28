// tests/motion/trapezoidal_trajectory_test.cpp
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
#include "robot/motion/trapezoidal_trajectory.hpp"

using robot::core::JointError;
using robot::core::JointLimits;
using robot::motion::TrapezoidalTrajectory;
using robot::motion::TrajectorySample;

namespace {

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0,
        .max_position     =  180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

}  // namespace

// --- create() validation ---

TEST(TrapezoidalTrajectory, RejectsNonPositiveVelocity) {
    auto limits = defaultLimits();
    limits.max_velocity = 0.0;

    auto result = TrapezoidalTrajectory::create(0.0, 10.0, limits);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::InvalidConfiguration);
}

TEST(TrapezoidalTrajectory, RejectsNonPositiveAcceleration) {
    auto limits = defaultLimits();
    limits.max_acceleration = -1.0;

    auto result = TrapezoidalTrajectory::create(0.0, 10.0, limits);

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::InvalidConfiguration);
}

TEST(TrapezoidalTrajectory, RejectsStartPositionOutOfRange) {
    auto result = TrapezoidalTrajectory::create(500.0, 10.0, defaultLimits());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::PositionOutOfRange);
}

TEST(TrapezoidalTrajectory, RejectsTargetPositionOutOfRange) {
    auto result = TrapezoidalTrajectory::create(0.0, 500.0, defaultLimits());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), JointError::PositionOutOfRange);
}

// --- Zero-distance trajectory ---

TEST(TrapezoidalTrajectory, ZeroDistanceHasZeroDurationAndStaysAtStart) {
    auto trajectory = TrapezoidalTrajectory::create(3.0, 3.0, defaultLimits()).value();

    EXPECT_EQ(trajectory.duration(), std::chrono::nanoseconds(0));

    for (auto t : {std::chrono::nanoseconds(0), std::chrono::nanoseconds(std::chrono::milliseconds(1)),
                   std::chrono::nanoseconds(std::chrono::seconds(5))}) {
        auto sample = trajectory.sample(t);
        EXPECT_DOUBLE_EQ(sample.position, 3.0);
        EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
        EXPECT_DOUBLE_EQ(sample.acceleration, 0.0);
    }
}

// --- Trapezoidal profile (distance=10, v=2, a=5 -> accel=0.4s, cruise=4.6s, total=5.4s) ---

class TrapezoidalProfile : public ::testing::Test {
protected:
    TrapezoidalTrajectory trajectory = TrapezoidalTrajectory::create(0.0, 10.0, defaultLimits()).value();
};

TEST_F(TrapezoidalProfile, StartsAtRestWithFullAcceleration) {
    auto sample = trajectory.sample(std::chrono::nanoseconds(0));
    EXPECT_DOUBLE_EQ(sample.position, 0.0);
    EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
    EXPECT_DOUBLE_EQ(sample.acceleration, 5.0);
}

TEST_F(TrapezoidalProfile, ReachesMaxVelocityAtEndOfAccelPhase) {
    auto sample = trajectory.sample(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(0.4)));
    EXPECT_NEAR(sample.velocity, 2.0, 1e-9);
}

TEST_F(TrapezoidalProfile, EndsExactlyAtTargetWithZeroVelocity) {
    auto sample = trajectory.sample(trajectory.duration());
    EXPECT_DOUBLE_EQ(sample.position, 10.0);
    EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
    EXPECT_DOUBLE_EQ(sample.acceleration, 0.0);
}

TEST_F(TrapezoidalProfile, DurationMatchesClosedFormComputation) {
    // accel = v/a = 2/5 = 0.4s; cruise = (10 - 2*(v^2/2a)) / v = (10-0.8)/2 = 4.6s; total = 5.4s
    // Tolerance is looser than raw double precision because duration()
    // round-trips through std::chrono::nanoseconds (see TrapezoidalTrajectory::sample()'s
    // "endpoint epsilon" comment for why sub-microsecond bias here is expected and harmless).
    EXPECT_NEAR(std::chrono::duration<double>(trajectory.duration()).count(), 5.4, 1e-6);
}

TEST_F(TrapezoidalProfile, PositionIsMonotonicNonDecreasing) {
    double previous = -1.0;
    const auto totalNs = trajectory.duration().count();
    for (std::int64_t ns = 0; ns <= totalNs; ns += totalNs / 200) {
        auto sample = trajectory.sample(std::chrono::nanoseconds(ns));
        EXPECT_GE(sample.position, previous - 1e-9);
        previous = sample.position;
    }
}

TEST_F(TrapezoidalProfile, ClampsTimeBeforeStart) {
    auto sample = trajectory.sample(std::chrono::nanoseconds(-1'000'000));
    EXPECT_DOUBLE_EQ(sample.position, 0.0);
    EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
}

TEST_F(TrapezoidalProfile, ClampsTimeAfterEnd) {
    auto sample = trajectory.sample(trajectory.duration() + std::chrono::seconds(10));
    EXPECT_DOUBLE_EQ(sample.position, 10.0);
    EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
    EXPECT_DOUBLE_EQ(sample.acceleration, 0.0);
}

// --- Triangular profile (distance=2, v=10 (unreachable), a=5) ---

class TriangularProfile : public ::testing::Test {
protected:
    JointLimits limits = JointLimits{
        .min_position = -180.0, .max_position = 180.0, .max_velocity = 10.0, .max_acceleration = 5.0,
    };
    TrapezoidalTrajectory trajectory = TrapezoidalTrajectory::create(0.0, 2.0, limits).value();
};

TEST_F(TriangularProfile, PeakVelocityStaysBelowMaxVelocity) {
    // peak_v = sqrt(D*a) = sqrt(2*5) = sqrt(10) ~= 3.1623, well below max_velocity = 10.
    auto midpoint = std::chrono::duration_cast<std::chrono::nanoseconds>(trajectory.duration()) / 2;
    auto sample = trajectory.sample(midpoint);
    EXPECT_NEAR(sample.velocity, std::sqrt(10.0), 1e-6);
    EXPECT_LT(sample.velocity, 10.0);
}

TEST_F(TriangularProfile, MidpointInTimeReachesHalfTheDistance) {
    auto midpoint = std::chrono::duration_cast<std::chrono::nanoseconds>(trajectory.duration()) / 2;
    auto sample = trajectory.sample(midpoint);
    EXPECT_NEAR(sample.position, 1.0, 1e-6);  // half of distance 2, by symmetry
}

TEST_F(TriangularProfile, EndsExactlyAtTarget) {
    auto sample = trajectory.sample(trajectory.duration());
    EXPECT_DOUBLE_EQ(sample.position, 2.0);
    EXPECT_DOUBLE_EQ(sample.velocity, 0.0);
}

// --- Negative direction ---

TEST(TrapezoidalTrajectory, NegativeDirectionHasNegativeVelocityAndReachesTarget) {
    auto trajectory = TrapezoidalTrajectory::create(5.0, 0.0, defaultLimits()).value();

    auto start = trajectory.sample(std::chrono::nanoseconds(0));
    EXPECT_DOUBLE_EQ(start.position, 5.0);
    EXPECT_DOUBLE_EQ(start.acceleration, -5.0);

    // Sample somewhere in the middle of the profile: velocity must be negative.
    auto mid = trajectory.sample(trajectory.duration() / 2);
    EXPECT_LT(mid.velocity, 0.0);

    auto end = trajectory.sample(trajectory.duration());
    EXPECT_DOUBLE_EQ(end.position, 0.0);
    EXPECT_DOUBLE_EQ(end.velocity, 0.0);
}
