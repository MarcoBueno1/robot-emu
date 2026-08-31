// src/sensors/encoder_sensor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/sensors/encoder_sensor.hpp"

#include <limits>

namespace robot::sensors {

using robot::hardware::HardwareError;
using robot::hardware::VirtualEncoder;

std::expected<EncoderSensor, HardwareError> EncoderSensor::create(VirtualEncoder encoder,
                                                                     double noiseStdDevRadians,
                                                                     double offsetRadians,
                                                                     std::uint64_t seed) noexcept {
    if (noiseStdDevRadians < 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return EncoderSensor(std::move(encoder), noiseStdDevRadians, offsetRadians, seed);
}

EncoderSensor::EncoderSensor(VirtualEncoder encoder, double noiseStdDevRadians, double offsetRadians,
                              std::uint64_t seed) noexcept
    : encoder_(std::move(encoder)),
      noiseStdDevRadians_(noiseStdDevRadians),
      offsetRadians_(offsetRadians),
      rng_(seed),
      noiseDist_(0.0, noiseStdDevRadians) {}

double EncoderSensor::read(double truePositionRadians) noexcept {
    if (failureMode_ == EncoderFailureMode::Disconnected) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (failureMode_ == EncoderFailureMode::Stuck) {
        return lastReading_;
    }

    const double quantized = encoder_.sample(truePositionRadians + offsetRadians_);
    const double noise = noiseStdDevRadians_ > 0.0 ? noiseDist_(rng_) : 0.0;
    const double reading = quantized + noise;

    lastReading_ = reading;
    return reading;
}

EncoderFailureMode EncoderSensor::failureMode() const noexcept {
    return failureMode_;
}

void EncoderSensor::setFailureMode(EncoderFailureMode mode) noexcept {
    failureMode_ = mode;
}

}  // namespace robot::sensors
