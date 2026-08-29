// src/protocol/tcp_listener.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
#include "robot/protocol/tcp_listener.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

namespace robot::protocol {

namespace {
constexpr int kListenBacklog = 16;
}

TcpListener::TcpListener(int fd, std::uint16_t port) noexcept : fd_(fd), port_(port) {}

TcpListener::~TcpListener() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

TcpListener::TcpListener(TcpListener&& other) noexcept : fd_(other.fd_), port_(other.port_) {
    other.fd_ = -1;
}

TcpListener& TcpListener::operator=(TcpListener&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = other.fd_;
        port_ = other.port_;
        other.fd_ = -1;
    }
    return *this;
}

std::expected<TcpListener, TransportError> TcpListener::create(std::uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return std::unexpected(TransportError::BindFailed);
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(fd);
        return std::unexpected(TransportError::BindFailed);
    }

    if (::listen(fd, kListenBacklog) < 0) {
        ::close(fd);
        return std::unexpected(TransportError::ListenFailed);
    }

    // Read back the actual bound port — needed when port == 0 (OS-assigned
    // ephemeral port), harmless (returns the same value) otherwise.
    sockaddr_in bound{};
    socklen_t boundLen = sizeof(bound);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &boundLen) < 0) {
        ::close(fd);
        return std::unexpected(TransportError::BindFailed);
    }

    return TcpListener(fd, ntohs(bound.sin_port));
}

std::uint16_t TcpListener::port() const noexcept {
    return port_;
}

std::expected<TcpConnection, TransportError> TcpListener::accept() {
    sockaddr_in clientAddress{};
    socklen_t clientLen = sizeof(clientAddress);
    const int clientFd = ::accept(fd_, reinterpret_cast<sockaddr*>(&clientAddress), &clientLen);
    if (clientFd < 0) {
        return std::unexpected(TransportError::AcceptFailed);
    }
    return TcpConnection(clientFd);
}

}  // namespace robot::protocol
