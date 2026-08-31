// src/fault/fault_injector.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/fault/fault_injector.hpp"

#include <algorithm>

namespace robot::fault {

void FaultInjector::onFault(FaultType type, Handler handler) {
    handlers_[static_cast<int>(type)] = std::move(handler);
}

void FaultInjector::inject(FaultType type, std::optional<std::size_t> jointIndex) {
    const ActiveFault fault{type, jointIndex};
    if (std::find(active_.begin(), active_.end(), fault) == active_.end()) {
        active_.push_back(fault);
    }

    if (auto it = handlers_.find(static_cast<int>(type)); it != handlers_.end()) {
        it->second(jointIndex);
    }
}

void FaultInjector::clear(FaultType type, std::optional<std::size_t> jointIndex) {
    const ActiveFault fault{type, jointIndex};
    std::erase(active_, fault);
}

void FaultInjector::clearAll() noexcept {
    active_.clear();
}

bool FaultInjector::isActive(FaultType type, std::optional<std::size_t> jointIndex) const {
    const ActiveFault fault{type, jointIndex};
    return std::find(active_.begin(), active_.end(), fault) != active_.end();
}

std::vector<ActiveFault> FaultInjector::activeFaults() const {
    return active_;
}

}  // namespace robot::fault
