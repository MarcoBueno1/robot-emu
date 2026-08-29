// include/robot/protocol/tcp_connection.hpp
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
#include <span>
#include <string>
#include "robot/protocol/transport_error.hpp"

namespace robot::protocol {

class TcpListener;  // for the friend declaration below

/// @brief An RAII handle around one connected TCP socket (POSIX only).
///
/// Move-only, like any RAII wrapper around a single OS resource — the
/// moved-from instance's file descriptor is reset so its destructor's
/// close() becomes a no-op. Knows nothing about robot::protocol::Frame —
/// it moves raw bytes only; see FrameCodec for the framing layer above it.
class TcpConnection {
public:
    /// @brief Closes the underlying socket if still open.
    ~TcpConnection();

    TcpConnection(const TcpConnection&) = delete;
    TcpConnection& operator=(const TcpConnection&) = delete;
    TcpConnection(TcpConnection&& other) noexcept;
    TcpConnection& operator=(TcpConnection&& other) noexcept;

    /// @brief Connects to host:port over TCP.
    /// @return A connected TcpConnection, or TransportError::ConnectFailed.
    [[nodiscard]] static std::expected<TcpConnection, TransportError> connectTo(const std::string& host,
                                                                                  std::uint16_t port);

    /// @brief Blocks until all of bytes has been sent.
    /// @return Success, or TransportError::SendFailed.
    [[nodiscard]] std::expected<void, TransportError> send(std::span<const std::byte> bytes);

    /// @brief Blocks until at least one byte is available.
    ///
    /// May return fewer bytes than buffer.size() — callers assembling a
    /// full frame from FrameCodec must be prepared to call this more than
    /// once. Performs no heap allocation itself; buffer is used as-is.
    /// @return Number of bytes read (1 <= n <= buffer.size()), or
    ///         TransportError::ConnectionClosed if the peer closed the
    ///         connection, or TransportError::ReceiveFailed on error.
    [[nodiscard]] std::expected<std::size_t, TransportError> receive(std::span<std::byte> buffer);

private:
    explicit TcpConnection(int fd) noexcept;

    int fd_ = -1;

    friend class TcpListener;
};

}  // namespace robot::protocol
