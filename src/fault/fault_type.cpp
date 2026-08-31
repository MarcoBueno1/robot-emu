// src/fault/fault_type.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/fault/fault_type.hpp"

namespace robot::fault {

std::string_view toString(FaultType type) noexcept {
    switch (type) {
        case FaultType::EncoderFailure:        return "EncoderFailure";
        case FaultType::MotorFailure:          return "MotorFailure";
        case FaultType::OverCurrent:           return "OverCurrent";
        case FaultType::OverTemperature:       return "OverTemperature";
        case FaultType::CommunicationTimeout:  return "CommunicationTimeout";
        case FaultType::PositionError:         return "PositionError";
        case FaultType::VelocityError:         return "VelocityError";
        case FaultType::BrakeFailure:          return "BrakeFailure";
        case FaultType::LimitSwitch:           return "LimitSwitch";
        case FaultType::EmergencyStop:         return "EmergencyStop";
        case FaultType::PowerFailure:          return "PowerFailure";
    }
    __builtin_unreachable();
}

}  // namespace robot::fault
