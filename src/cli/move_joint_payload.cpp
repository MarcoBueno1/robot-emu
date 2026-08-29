// src/cli/move_joint_payload.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/cli/move_joint_payload.hpp"

#include "byte_order_internal.hpp"

namespace robot::cli {

using namespace robot::cli::internal;

namespace {
constexpr std::size_t kEncodedSize = 1 + 8;  // u8 jointIndex + f64 targetRadians
}

std::vector<std::byte> encodeMoveJointPayload(const MoveJointPayload& payload) {
    std::vector<std::byte> out;
    out.reserve(kEncodedSize);
    out.push_back(static_cast<std::byte>(payload.jointIndex));
    writeF64BE(out, payload.targetRadians);
    return out;
}

std::expected<MoveJointPayload, CliError> decodeMoveJointPayload(std::span<const std::byte> bytes) {
    if (bytes.size() < kEncodedSize) {
        return std::unexpected(CliError::TruncatedPayload);
    }

    const auto jointIndex = std::to_integer<std::uint8_t>(bytes[0]);
    const double targetRadians = readF64BE(bytes.subspan(1, 8));

    return MoveJointPayload{jointIndex, targetRadians};
}

}  // namespace robot::cli
