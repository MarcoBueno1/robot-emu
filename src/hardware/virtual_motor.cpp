// src/hardware/virtual_motor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/hardware/virtual_motor.hpp"

#include <cmath>

namespace robot::hardware {

std::expected<VirtualMotor, HardwareError> VirtualMotor::create(double max_torque,
                                                                  std::chrono::nanoseconds time_constant) noexcept {
    if (max_torque <= 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    const double timeConstantS = std::chrono::duration<double>(time_constant).count();
    if (timeConstantS <= 0.0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return VirtualMotor(max_torque, timeConstantS);
}

VirtualMotor::VirtualMotor(double max_torque, double time_constant_s) noexcept
    : max_torque_(max_torque), time_constant_s_(time_constant_s) {}

double VirtualMotor::torque() const noexcept {
    return torque_;
}

double VirtualMotor::commandedTorque() const noexcept {
    return commanded_torque_;
}

std::expected<void, HardwareError> VirtualMotor::setCommandedTorque(double torque) noexcept {
    if (torque < -max_torque_ || torque > max_torque_) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    commanded_torque_ = torque;
    return {};
}

void VirtualMotor::update(std::chrono::nanoseconds dt) noexcept {
    const double dtS = std::chrono::duration<double>(dt).count();
    if (dtS <= 0.0) {
        return;
    }
    // Exact analytic solution of dTorque/dt = (commanded - torque) / tau,
    // valid for any dt (not a per-step approximation).
    const double alpha = std::exp(-dtS / time_constant_s_);
    torque_ = commanded_torque_ + (torque_ - commanded_torque_) * alpha;
}

}  // namespace robot::hardware
