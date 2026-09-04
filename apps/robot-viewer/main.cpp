// apps/robot-viewer/main.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// An external protocol client, exactly like robotctl — never linked into
// the core (docs/architecture.md section 3.14). Polls apps/robot-emulator
// via robot::cli::Client (Phase 7, unmodified) and relays each reading as
// JSON to one connected browser tab over a hand-rolled minimal WebSocket
// server (robot_websocket). Read-only: never sends a command back.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <sstream>
#include <thread>

#include "robot/cli/client.hpp"
#include "robot/protocol/tcp_listener.hpp"
#include "robot/websocket/websocket_connection.hpp"

using robot::cli::Client;
using robot::cli::Command;
using robot::cli::CommandKind;
using robot::protocol::TcpListener;
using robot::websocket::WebSocketConnection;

namespace {

constexpr std::uint16_t kDefaultEmulatorPort = 9000;
constexpr std::uint16_t kDefaultViewerPort = 9001;

std::string statusToJson(const robot::cli::StatusPayload& status) {
    std::ostringstream json;
    json << "{\"state\":\"" << status.controllerStateName << "\",\"joints\":[";
    for (std::size_t i = 0; i < status.joints.size(); ++i) {
        if (i > 0) json << ",";
        const double positionDeg = status.joints[i].positionRadians * 180.0 / std::numbers::pi;
        const double velocityDeg = status.joints[i].velocityRadiansPerSecond * 180.0 / std::numbers::pi;
        json << "{\"position_deg\":" << positionDeg << ",\"velocity_deg_s\":" << velocityDeg << "}";
    }
    json << "]}";
    return json.str();
}

}  // namespace

int main(int argc, char** argv) {
    const std::string emulatorHost = "127.0.0.1";
    const std::uint16_t emulatorPort =
        argc > 1 ? static_cast<std::uint16_t>(std::atoi(argv[1])) : kDefaultEmulatorPort;
    const std::uint16_t viewerPort = argc > 2 ? static_cast<std::uint16_t>(std::atoi(argv[2])) : kDefaultViewerPort;

    auto listener = TcpListener::create(viewerPort);
    if (!listener.has_value()) {
        std::cerr << "robot-viewer: failed to bind port " << viewerPort << "\n";
        return 1;
    }
    std::cout << "robot-viewer: listening for browser connections on ws://127.0.0.1:" << listener->port() << "\n";
    std::cout << "robot-viewer: open apps/robot-viewer/index.html in a browser to connect\n";

    while (true) {
        auto raw = listener->accept();
        if (!raw.has_value()) {
            continue;
        }
        auto ws = WebSocketConnection::accept(std::move(raw.value()));
        if (!ws.has_value()) {
            std::cerr << "robot-viewer: WebSocket handshake failed\n";
            continue;
        }
        std::cout << "robot-viewer: browser connected\n";

        auto client = Client::connect(emulatorHost, emulatorPort);
        if (!client.has_value()) {
            std::cerr << "robot-viewer: failed to connect to robot-emulator at " << emulatorHost << ":"
                       << emulatorPort << "\n";
            continue;
        }

        while (true) {
            auto outcome = client->execute(Command{.kind = CommandKind::Status});
            if (!outcome.has_value() || !outcome->statusPayload.has_value()) {
                break;  // emulator connection lost
            }

            auto sent = ws->sendText(statusToJson(outcome->statusPayload.value()));
            if (!sent.has_value()) {
                break;  // browser tab closed
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "robot-viewer: session ended\n";
    }
}
