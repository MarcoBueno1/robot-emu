// src/core/robot_config.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/core/robot_config.hpp"

namespace robot::core {

std::expected<void, JointError> RobotConfig::validate() const noexcept {
    if (joints.empty()) {
        return std::unexpected(JointError::InvalidConfiguration);
    }

    for (const auto& limits : joints) {
        auto result = limits.validate();
        if (!result.has_value()) {
            return result;
        }
    }

    return {};
}

}
