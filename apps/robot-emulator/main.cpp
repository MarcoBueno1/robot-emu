// apps/robot-emulator/main.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// The real server every "known gap" note since Phase 7 pointed to. See
// docs/task-briefs/server-integration.md — especially section 6.2 for why
// there is exactly one std::mutex here, guarding the shared Robot/
// ControllerStateMachine against the three threads that touch them: this
// server's own control thread, its connection-serving loop, and its
// watchdog trip-response thread.
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <numbers>
#include <thread>
#include <vector>

#include "robot/cli/move_joint_payload.hpp"
#include "robot/controller/controller_state_machine.hpp"
#include "robot/core/robot.hpp"
#include "robot/hardware/virtual_brake.hpp"
#include "robot/protocol/frame_codec.hpp"
#include "robot/protocol/tcp_listener.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/safety/emergency_stop_controller.hpp"
#include "robot/safety/watchdog.hpp"
#include "robot/server/command_dispatcher.hpp"

using robot::controller::ControllerEvent;
using robot::controller::ControllerStateMachine;
using robot::core::JointLimits;
using robot::core::Robot;
using robot::core::RobotConfig;
using robot::hardware::VirtualBrake;
using robot::protocol::FrameCodec;
using robot::protocol::TcpConnection;
using robot::protocol::TcpListener;
using robot::runtime::ControlLoop;
using robot::runtime::ControlLoopFrequency;
using robot::safety::EmergencyStopController;
using robot::safety::Watchdog;
using robot::server::CommandDispatcher;

namespace {

constexpr int kJointCount = 6;
constexpr std::uint16_t kDefaultPort = 9000;

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0 * std::numbers::pi / 180.0,
        .max_position     =  180.0 * std::numbers::pi / 180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

// Reads exactly one frame's worth of bytes off connection into received,
// looping receive() as needed — same pattern Phase 6/7's own tests used.
[[nodiscard]] bool receiveOneFrame(TcpConnection& connection, std::vector<std::byte>& received,
                                     FrameCodec::DecodeResult& outResult) {
    std::array<std::byte, 4096> chunk{};
    while (true) {
        auto decoded = FrameCodec::decode(received);
        if (decoded.has_value()) {
            outResult = std::move(decoded.value());
            return true;
        }
        if (decoded.error() != robot::protocol::ProtocolError::Incomplete) {
            return false;
        }
        auto n = connection.receive(chunk);
        if (!n.has_value()) {
            return false;
        }
        received.insert(received.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(*n));
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::uint16_t port = argc > 1 ? static_cast<std::uint16_t>(std::atoi(argv[1])) : kDefaultPort;

    // --- Build the scenario ---
    RobotConfig config{.name = "RE-6AXIS", .joints = std::vector<JointLimits>(kJointCount, defaultLimits())};
    auto robotResult = Robot::create(config);
    if (!robotResult.has_value()) {
        std::cerr << "robot-emulator: failed to create Robot\n";
        return 1;
    }
    Robot robot = std::move(robotResult.value());

    ControllerStateMachine controller;
    // Startup power-on sequence completes automatically — see the task
    // brief section 6.3: a real controller's boot sequence isn't something
    // an operator triggers per-command.
    for (auto event : {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete}) {
        static_cast<void>(controller.handleEvent(event));
    }

    std::array<VirtualBrake, kJointCount> brakes;
    EmergencyStopController estop(controller, brakes);
    CommandDispatcher dispatcher(robot, controller, estop);

    ControlLoop controlLoop(robot, controller, ControlLoopFrequency::Hz1000);

    // --- The one mutex guarding Robot/ControllerStateMachine — see
    //     docs/task-briefs/server-integration.md section 6.2. ---
    std::mutex robotMutex;

    // --- Control thread: reimplements ControlLoop::start()'s own pacing
    //     loop, since there is no hook to inject an external lock into
    //     that private internal thread. step() itself is exactly what
    //     Phase 3 designed to be safe to call directly, however a caller
    //     wants — this is that. ---
    std::jthread controlThread([&](std::stop_token stopToken) {
        auto next = std::chrono::steady_clock::now();
        while (!stopToken.stop_requested()) {
            {
                std::lock_guard<std::mutex> lock(robotMutex);
                controlLoop.step();
            }
            next += controlLoop.period();
            std::this_thread::sleep_until(next);
        }
    });

    // --- Watchdog: no lock needed for this specific read (ControlLoopMetrics
    //     fields are individual atomics, per Phase 3). ---
    Watchdog watchdog([&] { return controlLoop.metrics().cyclesExecuted; }, std::chrono::milliseconds(100),
                       std::chrono::milliseconds(10));
    if (!watchdog.start().has_value()) {
        std::cerr << "robot-emulator: failed to start watchdog\n";
        return 1;
    }

    std::jthread watchdogResponseThread([&](std::stop_token stopToken) {
        bool alreadyTripped = false;
        while (!stopToken.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (watchdog.tripped() && !alreadyTripped) {
                alreadyTripped = true;
                std::lock_guard<std::mutex> lock(robotMutex);
                estop.trigger();
                std::cerr << "robot-emulator: watchdog tripped — emergency stop triggered\n";
            }
        }
    });

    // --- Connection-serving loop: one client at a time (see Non-Goals) ---
    auto listener = TcpListener::create(port);
    if (!listener.has_value()) {
        std::cerr << "robot-emulator: failed to bind port " << port << "\n";
        return 1;
    }
    std::cout << "robot-emulator: listening on port " << listener->port() << "\n";

    while (true) {
        auto connection = listener->accept();
        if (!connection.has_value()) {
            std::cerr << "robot-emulator: accept() failed, continuing\n";
            continue;
        }
        std::cout << "robot-emulator: client connected\n";

        std::vector<std::byte> received;
        while (true) {
            FrameCodec::DecodeResult decoded;
            if (!receiveOneFrame(connection.value(), received, decoded)) {
                break;  // client disconnected or a transport/protocol error
            }
            received.erase(received.begin(), received.begin() + static_cast<std::ptrdiff_t>(decoded.bytesConsumed));

            robot::protocol::Frame response;
            {
                std::lock_guard<std::mutex> lock(robotMutex);
                response = dispatcher.dispatch(decoded.frame);
            }

            auto encoded = FrameCodec::encode(response);
            if (!connection->send(encoded).has_value()) {
                break;
            }
        }
        std::cout << "robot-emulator: client disconnected\n";
    }
}
