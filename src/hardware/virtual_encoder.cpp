// src/hardware/virtual_encoder.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/hardware/virtual_encoder.hpp"

#include <cmath>
#include <numbers>

namespace robot::hardware {

std::expected<VirtualEncoder, HardwareError> VirtualEncoder::create(std::uint32_t counts_per_revolution) noexcept {
    if (counts_per_revolution == 0) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return VirtualEncoder(counts_per_revolution);
}

VirtualEncoder::VirtualEncoder(std::uint32_t counts_per_revolution) noexcept
    : counts_per_revolution_(counts_per_revolution) {}

double VirtualEncoder::resolutionRadians() const noexcept {
    return (2.0 * std::numbers::pi) / static_cast<double>(counts_per_revolution_);
}

std::int64_t VirtualEncoder::counts(double true_position_radians) const noexcept {
    // std::llround rounds half-away-from-zero correctly for both positive
    // and negative inputs — a hand-rolled static_cast<int64_t>(x + 0.5)
    // rounds negative values incorrectly.
    return std::llround(true_position_radians / resolutionRadians());
}

double VirtualEncoder::sample(double true_position_radians) const noexcept {
    return static_cast<double>(counts(true_position_radians)) * resolutionRadians();
}

}  // namespace robot::hardware
