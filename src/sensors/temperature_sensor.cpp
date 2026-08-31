// src/sensors/temperature_sensor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/sensors/temperature_sensor.hpp"

#include <cmath>

namespace robot::sensors {

using robot::hardware::HardwareError;

std::expected<TemperatureSensor, HardwareError> TemperatureSensor::create(
    double ambientCelsius, std::chrono::nanoseconds timeConstant, double maxSafeCelsius) noexcept {
    const double timeConstantS = std::chrono::duration<double>(timeConstant).count();
    if (timeConstantS <= 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return TemperatureSensor(ambientCelsius, timeConstantS, maxSafeCelsius);
}

TemperatureSensor::TemperatureSensor(double celsius, double timeConstantS, double maxSafeCelsius) noexcept
    : celsius_(celsius), timeConstantS_(timeConstantS), maxSafeCelsius_(maxSafeCelsius) {}

double TemperatureSensor::celsius() const noexcept {
    return celsius_;
}

bool TemperatureSensor::isOverTemperature() const noexcept {
    return celsius_ > maxSafeCelsius_;
}

void TemperatureSensor::update(std::chrono::nanoseconds dt, double targetCelsius) noexcept {
    const double dtS = std::chrono::duration<double>(dt).count();
    if (dtS <= 0.0) {
        return;
    }
    // Exact analytic solution, same technique as VirtualMotor::update() (Phase 5).
    const double alpha = std::exp(-dtS / timeConstantS_);
    celsius_ = targetCelsius + (celsius_ - targetCelsius) * alpha;
}

}  // namespace robot::sensors
