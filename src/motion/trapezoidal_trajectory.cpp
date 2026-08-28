// src/motion/trapezoidal_trajectory.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/motion/trapezoidal_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace robot::motion {

using robot::core::JointError;
using robot::core::JointLimits;

std::expected<TrapezoidalTrajectory, JointError> TrapezoidalTrajectory::create(
    double start_position, double target_position, const JointLimits& limits) noexcept {
    if (auto validation = limits.validate(); !validation.has_value()) {
        return std::unexpected(validation.error());
    }
    if (start_position < limits.min_position || start_position > limits.max_position ||
        target_position < limits.min_position || target_position > limits.max_position) {
        return std::unexpected(JointError::PositionOutOfRange);
    }

    const double distance = target_position - start_position;
    const double absDistance = std::abs(distance);
    const double sign = absDistance > 0.0 ? (distance > 0.0 ? 1.0 : -1.0) : 0.0;
    const double v = limits.max_velocity;
    const double a = limits.max_acceleration;

    if (absDistance == 0.0) {
        return TrapezoidalTrajectory(start_position, target_position, sign, a,
                                      /*accel_duration_s=*/0.0, /*cruise_duration_s=*/0.0,
                                      /*cruise_velocity=*/0.0, /*accel_distance=*/0.0);
    }

    const double accelDistanceAtMaxV = (v * v) / (2.0 * a);  // distance covered accelerating 0 -> v

    if (2.0 * accelDistanceAtMaxV <= absDistance) {
        // Trapezoidal: reaches max_velocity, has a cruise phase.
        const double accelDurationS = v / a;
        const double cruiseDistance = absDistance - 2.0 * accelDistanceAtMaxV;
        const double cruiseDurationS = cruiseDistance / v;
        return TrapezoidalTrajectory(start_position, target_position, sign, a, accelDurationS,
                                      cruiseDurationS, /*cruise_velocity=*/v, accelDistanceAtMaxV);
    }

    // Triangular: never reaches max_velocity, no cruise phase.
    const double peakVelocity = std::sqrt(absDistance * a);
    const double accelDurationS = peakVelocity / a;
    const double accelDistance = 0.5 * a * accelDurationS * accelDurationS;
    return TrapezoidalTrajectory(start_position, target_position, sign, a, accelDurationS,
                                  /*cruise_duration_s=*/0.0, /*cruise_velocity=*/peakVelocity, accelDistance);
}

TrapezoidalTrajectory::TrapezoidalTrajectory(double start_position, double target_position, double sign,
                                              double max_acceleration, double accel_duration_s,
                                              double cruise_duration_s, double cruise_velocity,
                                              double accel_distance) noexcept
    : start_position_(start_position),
      target_position_(target_position),
      sign_(sign),
      max_acceleration_(max_acceleration),
      accel_duration_s_(accel_duration_s),
      cruise_duration_s_(cruise_duration_s),
      cruise_velocity_(cruise_velocity),
      accel_distance_(accel_distance) {}

std::chrono::nanoseconds TrapezoidalTrajectory::duration() const noexcept {
    const double totalS = 2.0 * accel_duration_s_ + cruise_duration_s_;
    return std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>(totalS));
}

TrajectorySample TrapezoidalTrajectory::sample(std::chrono::nanoseconds t) const noexcept {
    if (sign_ == 0.0) {
        return TrajectorySample{start_position_, 0.0, 0.0};
    }

    // A tiny tolerance absorbs the inherent floating-point/nanosecond-
    // rounding bias between duration()'s quantized nanoseconds and the
    // exact double-precision phase durations computed in create() — e.g.
    // sample(duration()) landing 1ns "before" the exact end due to
    // truncation/rounding during the seconds->nanoseconds conversion.
    // 100ns is far below any control-loop period of interest in this
    // project (the fastest configured frequency, 10kHz, has a 100us
    // period — 1000x larger), so this cannot mask a real, meaningful
    // difference in intended sample time.
    constexpr double kEndpointEpsilonS = 1e-7;  // 100ns

    const double totalDurationS = 2.0 * accel_duration_s_ + cruise_duration_s_;
    double tau = std::chrono::duration<double>(t).count();
    tau = std::clamp(tau, 0.0, totalDurationS);

    if (tau >= totalDurationS - kEndpointEpsilonS) {
        return TrajectorySample{target_position_, 0.0, 0.0};
    }

    const double a = max_acceleration_;

    if (tau <= accel_duration_s_) {
        // Acceleration phase.
        const double vel = a * tau;
        const double pos = 0.5 * a * tau * tau;
        return TrajectorySample{start_position_ + sign_ * pos, sign_ * vel, sign_ * a};
    }

    if (tau <= accel_duration_s_ + cruise_duration_s_) {
        // Cruise phase (only reached when cruise_duration_s_ > 0, i.e. trapezoidal).
        const double tIntoCruise = tau - accel_duration_s_;
        const double pos = accel_distance_ + cruise_velocity_ * tIntoCruise;
        return TrajectorySample{start_position_ + sign_ * pos, sign_ * cruise_velocity_, 0.0};
    }

    // Deceleration phase.
    const double tIntoDec = tau - accel_duration_s_ - cruise_duration_s_;
    const double cruiseDistance = cruise_velocity_ * cruise_duration_s_;
    const double vel = cruise_velocity_ - a * tIntoDec;
    const double pos = accel_distance_ + cruiseDistance + cruise_velocity_ * tIntoDec - 0.5 * a * tIntoDec * tIntoDec;
    return TrajectorySample{start_position_ + sign_ * pos, sign_ * vel, -sign_ * a};
}

}  // namespace robot::motion
