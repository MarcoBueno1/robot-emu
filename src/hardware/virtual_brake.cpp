// src/hardware/virtual_brake.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/hardware/virtual_brake.hpp"

namespace robot::hardware {

VirtualBrake::VirtualBrake() noexcept = default;

BrakeState VirtualBrake::state() const noexcept {
    return state_;
}

bool VirtualBrake::isEngaged() const noexcept {
    return state_ == BrakeState::Engaged;
}

void VirtualBrake::engage() noexcept {
    state_ = BrakeState::Engaged;
}

void VirtualBrake::release() noexcept {
    state_ = BrakeState::Released;
}

}  // namespace robot::hardware
