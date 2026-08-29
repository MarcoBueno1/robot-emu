// src/hardware/digital_io.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/hardware/digital_io.hpp"

namespace robot::hardware {

DigitalIO::DigitalIO(std::size_t channel_count) noexcept : channels_(channel_count, false) {}

std::size_t DigitalIO::channelCount() const noexcept {
    return channels_.size();
}

std::expected<bool, HardwareError> DigitalIO::read(std::size_t channel) const noexcept {
    if (channel >= channels_.size()) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    return channels_[channel];
}

std::expected<void, HardwareError> DigitalIO::write(std::size_t channel, bool value) noexcept {
    if (channel >= channels_.size()) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    channels_[channel] = value;
    return {};
}

}  // namespace robot::hardware
