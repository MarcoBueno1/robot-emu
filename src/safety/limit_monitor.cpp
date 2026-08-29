// src/safety/limit_monitor.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/safety/limit_monitor.hpp"

#include <algorithm>

namespace robot::safety {

LimitMonitor::LimitMonitor(double marginRadians) noexcept : marginRadians_(marginRadians) {}

std::vector<LimitViolation> LimitMonitor::check(const robot::core::Robot& robot) const {
    std::vector<LimitViolation> violations;

    const auto joints = robot.joints();
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto& joint = joints[i];
        const auto& limits = joint.limits();
        const double position = joint.position();

        const double distanceToMin = position - limits.min_position;
        const double distanceToMax = limits.max_position - position;
        const double distanceToNearest = std::min(distanceToMin, distanceToMax);

        if (distanceToNearest <= marginRadians_) {
            violations.push_back(LimitViolation{i, position, std::max(distanceToNearest, 0.0)});
        }
    }

    return violations;
}

}  // namespace robot::safety
