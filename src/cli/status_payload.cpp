// src/cli/status_payload.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/cli/status_payload.hpp"

#include "byte_order_internal.hpp"

namespace robot::cli {

using namespace robot::cli::internal;

std::vector<std::byte> encodeStatusPayload(const StatusPayload& status) {
    std::vector<std::byte> out;

    writeU16BE(out, static_cast<std::uint16_t>(status.robotName.size()));
    for (char c : status.robotName) {
        out.push_back(static_cast<std::byte>(c));
    }

    writeU16BE(out, static_cast<std::uint16_t>(status.controllerStateName.size()));
    for (char c : status.controllerStateName) {
        out.push_back(static_cast<std::byte>(c));
    }

    writeU16BE(out, static_cast<std::uint16_t>(status.joints.size()));
    for (const auto& joint : status.joints) {
        writeF64BE(out, joint.positionRadians);
        writeF64BE(out, joint.velocityRadiansPerSecond);
    }

    return out;
}

std::expected<StatusPayload, CliError> decodeStatusPayload(std::span<const std::byte> bytes) {
    std::size_t offset = 0;

    auto needBytes = [&](std::size_t n) -> std::expected<void, CliError> {
        if (bytes.size() - offset < n) {
            return std::unexpected(CliError::TruncatedPayload);
        }
        return {};
    };

    if (auto check = needBytes(2); !check.has_value()) {
        return std::unexpected(check.error());
    }
    const std::uint16_t nameLength = readU16BE(bytes.subspan(offset, 2));
    offset += 2;

    if (auto check = needBytes(nameLength); !check.has_value()) {
        return std::unexpected(check.error());
    }
    std::string robotName(reinterpret_cast<const char*>(bytes.data() + offset), nameLength);
    offset += nameLength;

    if (auto check = needBytes(2); !check.has_value()) {
        return std::unexpected(check.error());
    }
    const std::uint16_t stateNameLength = readU16BE(bytes.subspan(offset, 2));
    offset += 2;

    if (auto check = needBytes(stateNameLength); !check.has_value()) {
        return std::unexpected(check.error());
    }
    std::string stateName(reinterpret_cast<const char*>(bytes.data() + offset), stateNameLength);
    offset += stateNameLength;

    if (auto check = needBytes(2); !check.has_value()) {
        return std::unexpected(check.error());
    }
    const std::uint16_t jointCount = readU16BE(bytes.subspan(offset, 2));
    offset += 2;

    std::vector<JointStatus> joints;
    joints.reserve(jointCount);
    for (std::uint16_t i = 0; i < jointCount; ++i) {
        if (auto check = needBytes(16); !check.has_value()) {
            return std::unexpected(check.error());
        }
        const double position = readF64BE(bytes.subspan(offset, 8));
        const double velocity = readF64BE(bytes.subspan(offset + 8, 8));
        offset += 16;
        joints.push_back(JointStatus{position, velocity});
    }

    return StatusPayload{std::move(robotName), std::move(stateName), std::move(joints)};
}

}  // namespace robot::cli
