// include/robot/protocol/tcp_listener.hpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#pragma once
#include <cstdint>
#include <expected>
#include "robot/protocol/tcp_connection.hpp"
#include "robot/protocol/transport_error.hpp"

namespace robot::protocol {

/// @brief An RAII handle around a listening TCP socket (POSIX only).
///
/// Move-only, same rationale as TcpConnection. Accepts one client
/// connection at a time — see docs/task-briefs/phase-06-protocol.md
/// Non-Goals for why a multi-client server is out of scope here.
class TcpListener {
public:
    ~TcpListener();

    TcpListener(const TcpListener&) = delete;
    TcpListener& operator=(const TcpListener&) = delete;
    TcpListener(TcpListener&& other) noexcept;
    TcpListener& operator=(TcpListener&& other) noexcept;

    /// @brief Binds and listens on port.
    /// @param port TCP port to bind. Pass 0 to let the OS assign an
    ///        ephemeral free port — read the real bound port back via
    ///        port(). This is how tests avoid hardcoding (and colliding
    ///        on) a fixed port number.
    /// @return A listening TcpListener, or TransportError::BindFailed /
    ///         TransportError::ListenFailed.
    [[nodiscard]] static std::expected<TcpListener, TransportError> create(std::uint16_t port);

    /// @brief The actual bound port (resolved via the OS if 0 was requested).
    [[nodiscard]] std::uint16_t port() const noexcept;

    /// @brief Blocks until one client connects.
    /// @return The accepted connection, or TransportError::AcceptFailed.
    [[nodiscard]] std::expected<TcpConnection, TransportError> accept();

private:
    TcpListener(int fd, std::uint16_t port) noexcept;

    int fd_ = -1;
    std::uint16_t port_ = 0;
};

}  // namespace robot::protocol
