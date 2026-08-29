// src/protocol/tcp_connection.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/protocol/tcp_connection.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace robot::protocol {

TcpConnection::TcpConnection(int fd) noexcept : fd_(fd) {}

TcpConnection::~TcpConnection() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

TcpConnection::TcpConnection(TcpConnection&& other) noexcept : fd_(other.fd_) {
    other.fd_ = -1;
}

TcpConnection& TcpConnection::operator=(TcpConnection&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

std::expected<TcpConnection, TransportError> TcpConnection::connectTo(const std::string& host,
                                                                        std::uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return std::unexpected(TransportError::ConnectFailed);
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    // Numeric IPv4 addresses only (e.g. "127.0.0.1") — no DNS resolution.
    // Sufficient for this phase's loopback-only usage; see the task brief.
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
        ::close(fd);
        return std::unexpected(TransportError::ConnectFailed);
    }

    if (::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(fd);
        return std::unexpected(TransportError::ConnectFailed);
    }

    return TcpConnection(fd);
}

std::expected<void, TransportError> TcpConnection::send(std::span<const std::byte> bytes) {
    std::size_t sent = 0;
    while (sent < bytes.size()) {
        const ssize_t n = ::send(fd_, bytes.data() + sent, bytes.size() - sent, 0);
        if (n < 0) {
            return std::unexpected(TransportError::SendFailed);
        }
        sent += static_cast<std::size_t>(n);
    }
    return {};
}

std::expected<std::size_t, TransportError> TcpConnection::receive(std::span<std::byte> buffer) {
    const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
    if (n == 0) {
        return std::unexpected(TransportError::ConnectionClosed);
    }
    if (n < 0) {
        return std::unexpected(TransportError::ReceiveFailed);
    }
    return static_cast<std::size_t>(n);
}

}  // namespace robot::protocol
