// src/cli/format.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/cli/format.hpp"

#include <format>
#include <numbers>

namespace robot::cli {

namespace {
[[nodiscard]] double toDegrees(double radians) noexcept {
    return radians * 180.0 / std::numbers::pi;
}
}  // namespace

std::string formatStatusTable(const StatusPayload& status) {
    std::string out;
    out += std::format("Robot: {}\n", status.robotName);
    out += std::format("State: {}\n\n", status.controllerStateName);
    out += "Joint   Position   Velocity\n";
    out += "----------------------------\n";

    for (std::size_t i = 0; i < status.joints.size(); ++i) {
        const auto& joint = status.joints[i];
        out += std::format("J{:<6} {:>7.2f}\u00b0  {:>7.2f}\n", i + 1, toDegrees(joint.positionRadians),
                            toDegrees(joint.velocityRadiansPerSecond));
    }

    return out;
}

}  // namespace robot::cli
