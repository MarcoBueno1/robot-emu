// include/robot/cli/client.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <expected>
#include <optional>
#include <string>
#include "robot/cli/cli_error.hpp"
#include "robot/cli/command.hpp"
#include "robot/cli/response.hpp"
#include "robot/cli/status_payload.hpp"
#include "robot/protocol/tcp_connection.hpp"

namespace robot::cli {

/// @brief Connects to a robot server and executes one Command per call.
///
/// Owns one robot::protocol::TcpConnection. See
/// docs/task-briefs/phase-07-cli.md for the response wire convention this
/// class assumes the server on the other end follows — no such server
/// exists yet in this repository (see the brief's Non-Goals).
class Client {
public:
    /// @brief Connects to host:port over TCP.
    /// @return A connected Client, or CliError::TransportFailure.
    [[nodiscard]] static std::expected<Client, CliError> connect(const std::string& host, std::uint16_t port);

    /// @brief Result of a successfully executed command.
    struct Outcome {
        ResponseStatus status;
        /// Set only when command.kind == CommandKind::Status and status ==
        /// ResponseStatus::Ok.
        std::optional<StatusPayload> statusPayload;
    };

    /// @brief Encodes command, sends it, and blocks for one response Frame.
    /// @return The Outcome, or a CliError: TransportFailure /
    ///         ProtocolFailure (couldn't get/decode a response frame at
    ///         all), UnexpectedResponseType (response CommandType didn't
    ///         echo the request's), or ServerReportedError (response
    ///         ResponseStatus was Error).
    [[nodiscard]] std::expected<Outcome, CliError> execute(const Command& command);

private:
    explicit Client(robot::protocol::TcpConnection connection) noexcept;

    robot::protocol::TcpConnection connection_;
};

}  // namespace robot::cli
