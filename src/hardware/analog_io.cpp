// src/hardware/analog_io.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/hardware/analog_io.hpp"

namespace robot::hardware {

std::expected<AnalogIO, HardwareError> AnalogIO::create(std::size_t channel_count, double min_value,
                                                          double max_value) noexcept {
    if (min_value >= max_value) {
        return std::unexpected(HardwareError::InvalidConfiguration);
    }
    return AnalogIO(channel_count, min_value, max_value);
}

AnalogIO::AnalogIO(std::size_t channel_count, double min_value, double max_value) noexcept
    : min_value_(min_value), max_value_(max_value), channels_(channel_count, min_value) {}

std::size_t AnalogIO::channelCount() const noexcept {
    return channels_.size();
}

std::expected<double, HardwareError> AnalogIO::read(std::size_t channel) const noexcept {
    if (channel >= channels_.size()) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    return channels_[channel];
}

std::expected<void, HardwareError> AnalogIO::write(std::size_t channel, double value) noexcept {
    if (channel >= channels_.size()) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    if (value < min_value_ || value > max_value_) {
        return std::unexpected(HardwareError::OutOfRange);
    }
    channels_[channel] = value;
    return {};
}

}  // namespace robot::hardware
