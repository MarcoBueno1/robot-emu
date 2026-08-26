// src/core/robot.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/core/robot.hpp"

#include <utility>

namespace robot::core {

Robot::Robot(std::string name, std::vector<Joint> joints) noexcept
    : name_(std::move(name)), joints_(std::move(joints)) {}

std::expected<Robot, JointError> Robot::create(RobotConfig config) {
    auto validation = config.validate();
    if (!validation.has_value()) {
        return std::unexpected(validation.error());
    }

    std::vector<Joint> joints;
    joints.reserve(config.joints.size());
    for (const auto& limits : config.joints) {
        joints.emplace_back(limits);
    }

    return Robot(std::move(config.name), std::move(joints));
}

void Robot::update(std::chrono::nanoseconds dt) noexcept {
    for (auto& joint : joints_) {
        joint.update(dt);
    }
}

}
