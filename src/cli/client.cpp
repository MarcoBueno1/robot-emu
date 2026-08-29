// src/cli/client.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/cli/client.hpp"

#include <numbers>
#include "robot/cli/move_joint_payload.hpp"
#include "robot/protocol/frame_codec.hpp"

namespace robot::cli {

using robot::protocol::CommandType;
using robot::protocol::Frame;
using robot::protocol::FrameCodec;
using robot::protocol::TcpConnection;

namespace {

[[nodiscard]] CommandType toCommandType(CommandKind kind) noexcept {
    switch (kind) {
        case CommandKind::Status:        return CommandType::GetStatus;
        case CommandKind::Enable:        return CommandType::Enable;
        case CommandKind::Disable:       return CommandType::Disable;
        case CommandKind::Home:          return CommandType::Home;
        case CommandKind::MoveJoint:     return CommandType::MoveJoint;
        case CommandKind::Stop:          return CommandType::Stop;
        case CommandKind::EmergencyStop: return CommandType::EmergencyStop;
    }
    __builtin_unreachable();
}

[[nodiscard]] Frame buildRequestFrame(const Command& command) {
    Frame frame;
    frame.flags = 0;
    frame.type = toCommandType(command.kind);

    if (command.kind == CommandKind::MoveJoint) {
        const double targetRadians = command.targetDegrees * std::numbers::pi / 180.0;
        frame.payload = encodeMoveJointPayload(MoveJointPayload{command.jointIndex, targetRadians});
    }

    return frame;
}

}  // namespace

Client::Client(TcpConnection connection) noexcept : connection_(std::move(connection)) {}

std::expected<Client, CliError> Client::connect(const std::string& host, std::uint16_t port) {
    auto connection = TcpConnection::connectTo(host, port);
    if (!connection.has_value()) {
        return std::unexpected(CliError::TransportFailure);
    }
    return Client(std::move(connection.value()));
}

std::expected<Client::Outcome, CliError> Client::execute(const Command& command) {
    const Frame request = buildRequestFrame(command);
    const auto encoded = FrameCodec::encode(request);

    if (auto sent = connection_.send(encoded); !sent.has_value()) {
        return std::unexpected(CliError::TransportFailure);
    }

    // Receive until FrameCodec can decode a full response frame.
    std::vector<std::byte> received;
    std::array<std::byte, 4096> chunk{};
    std::expected<FrameCodec::DecodeResult, robot::protocol::ProtocolError> decoded =
        std::unexpected(robot::protocol::ProtocolError::Incomplete);

    while (true) {
        decoded = FrameCodec::decode(received);
        if (decoded.has_value() || decoded.error() != robot::protocol::ProtocolError::Incomplete) {
            break;
        }
        auto n = connection_.receive(chunk);
        if (!n.has_value()) {
            return std::unexpected(CliError::TransportFailure);
        }
        received.insert(received.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(*n));
    }

    if (!decoded.has_value()) {
        return std::unexpected(CliError::ProtocolFailure);
    }

    const Frame& response = decoded->frame;
    if (response.type != request.type) {
        return std::unexpected(CliError::UnexpectedResponseType);
    }
    if (response.payload.empty()) {
        return std::unexpected(CliError::ProtocolFailure);
    }

    const auto status = static_cast<ResponseStatus>(std::to_integer<std::uint8_t>(response.payload[0]));
    if (status == ResponseStatus::Error) {
        return std::unexpected(CliError::ServerReportedError);
    }

    Outcome outcome{status, std::nullopt};
    if (command.kind == CommandKind::Status) {
        std::span<const std::byte> statusBytes(response.payload.data() + 1, response.payload.size() - 1);
        auto statusPayload = decodeStatusPayload(statusBytes);
        if (!statusPayload.has_value()) {
            return std::unexpected(statusPayload.error());
        }
        outcome.statusPayload = std::move(statusPayload.value());
    }

    return outcome;
}

}  // namespace robot::cli
