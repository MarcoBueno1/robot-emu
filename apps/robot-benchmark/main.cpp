// apps/robot-benchmark/main.cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
// Real, measured benchmark per docs/architecture.md section 3.13 — "Real,
// measured numbers — never estimated." See
// docs/task-briefs/phase-11-benchmark.md for full methodology notes.
//
// This benchmark measures ControlLoop::step()'s own computational cost by
// calling it directly, synchronously, in a tight loop — no sleep_until
// pacing — which isolates the work being measured from OS scheduling
// noise. Deadline misses are 0 by construction (nothing here is paced),
// stated explicitly in the report rather than reusing ControlLoop's own
// (differently-scoped) metric.
#include <sys/resource.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numbers>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "robot/controller/controller_state_machine.hpp"
#include "robot/core/robot.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/sensors/current_sensor.hpp"
#include "robot/sensors/encoder_sensor.hpp"
#include "robot/sensors/proximity_sensor.hpp"
#include "robot/sensors/temperature_sensor.hpp"

using robot::controller::ControllerEvent;
using robot::controller::ControllerStateMachine;
using robot::core::JointLimits;
using robot::core::Robot;
using robot::core::RobotConfig;
using robot::runtime::ControlLoop;
using robot::runtime::ControlLoopFrequency;
using robot::sensors::CurrentSensor;
using robot::sensors::EncoderSensor;
using robot::sensors::ProximitySensor;
using robot::sensors::TemperatureSensor;

namespace {

constexpr int kJointCount = 6;
constexpr int kCyclesMeasured = 100'000;
constexpr int kCyclesWarmup = 2'000;

JointLimits defaultLimits() {
    return JointLimits{
        .min_position     = -180.0 * std::numbers::pi / 180.0,
        .max_position     =  180.0 * std::numbers::pi / 180.0,
        .max_velocity     =    2.0,
        .max_acceleration =    5.0,
    };
}

[[nodiscard]] std::string readCpuModel() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
        if (line.starts_with("model name")) {
            auto colon = line.find(':');
            if (colon != std::string::npos) {
                std::string model = line.substr(colon + 1);
                auto firstNonSpace = model.find_first_not_of(' ');
                return firstNonSpace == std::string::npos ? model : model.substr(firstNonSpace);
            }
        }
    }
    return "unknown CPU";
}

[[nodiscard]] std::string readOsPrettyName() {
    std::ifstream osRelease("/etc/os-release");
    std::string line;
    while (std::getline(osRelease, line)) {
        if (line.starts_with("PRETTY_NAME=")) {
            std::string value = line.substr(std::string("PRETTY_NAME=").size());
            if (!value.empty() && value.front() == '"') value.erase(0, 1);
            if (!value.empty() && value.back() == '"') value.pop_back();
            return value;
        }
    }
    return "unknown OS";
}

[[nodiscard]] double peakResidentMemoryMb() {
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    // ru_maxrss is in kilobytes on Linux.
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
}

struct SensorSet {
    EncoderSensor encoder;
    TemperatureSensor temperature;
    CurrentSensor current;
    ProximitySensor proximity;
};

}  // namespace

int main() {
    // --- Build the scenario: 6 joints, controller driven to Moving ---
    RobotConfig config{.name = "RE-6AXIS", .joints = std::vector<JointLimits>(kJointCount, defaultLimits())};
    auto robotResult = Robot::create(config);
    if (!robotResult.has_value()) {
        std::cerr << "robot-benchmark: failed to create Robot\n";
        return 1;
    }
    Robot robot = std::move(robotResult.value());

    ControllerStateMachine controller;
    for (auto event : {ControllerEvent::PowerOn, ControllerEvent::BootComplete, ControllerEvent::InitComplete,
                        ControllerEvent::ServoEnable, ControllerEvent::ControllerReady}) {
        auto result = controller.handleEvent(event);
        if (!result.has_value()) {
            std::cerr << "robot-benchmark: unexpected controller transition failure\n";
            return 1;
        }
    }

    for (auto& joint : robot.joints()) {
        joint.enable();
        auto set = joint.setTargetPosition(1.0);  // nonzero — real convergence work every cycle
        if (!set.has_value()) {
            std::cerr << "robot-benchmark: failed to set joint target\n";
            return 1;
        }
    }

    if (!controller.handleEvent(ControllerEvent::CommandMove).has_value()) {
        std::cerr << "robot-benchmark: failed to enter Moving\n";
        return 1;
    }

    ControlLoop controlLoop(robot, controller, ControlLoopFrequency::Hz1000);

    // --- 24 sensors: one of each of the 4 kinds, per joint ---
    std::vector<SensorSet> sensors;
    sensors.reserve(kJointCount);
    for (int i = 0; i < kJointCount; ++i) {
        auto encoder = robot::hardware::VirtualEncoder::create(4096).value();
        sensors.push_back(SensorSet{
            EncoderSensor::create(encoder, 0.0, 0.0, static_cast<std::uint64_t>(i)).value(),
            TemperatureSensor::create(25.0, std::chrono::milliseconds(500), 90.0).value(),
            CurrentSensor::create(10.0).value(),
            ProximitySensor::create(0.05).value(),
        });
    }

    const double positionBefore = robot.joint(0).position();

    // --- Warm-up (not measured): avoid first-touch/cache-cold effects skewing the report ---
    double sink = 0.0;
    for (int i = 0; i < kCyclesWarmup; ++i) {
        controlLoop.step();
        for (int j = 0; j < kJointCount; ++j) {
            sink += sensors[static_cast<std::size_t>(j)].encoder.read(
                robot.joint(static_cast<std::size_t>(j)).position());
        }
    }

    // --- Measured loop ---
    std::vector<std::chrono::nanoseconds> samples;
    samples.reserve(kCyclesMeasured);

    for (int i = 0; i < kCyclesMeasured; ++i) {
        const auto cycleStart = std::chrono::steady_clock::now();

        controlLoop.step();
        for (int j = 0; j < kJointCount; ++j) {
            auto& s = sensors[static_cast<std::size_t>(j)];
            auto& joint = robot.joint(static_cast<std::size_t>(j));
            sink += s.encoder.read(joint.position());
            s.temperature.update(controlLoop.period(), 25.0 + joint.velocity() * 5.0);
            sink += s.current.isOverCurrent(joint.velocity()) ? 1.0 : 0.0;
            sink += s.proximity.isTriggered(std::abs(joint.position())) ? 1.0 : 0.0;
        }

        const auto cycleEnd = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(cycleEnd - cycleStart));
    }

    const double positionAfter = robot.joint(0).position();

    // --- Compute statistics ---
    std::sort(samples.begin(), samples.end());

    const auto sumNs = std::accumulate(samples.begin(), samples.end(), std::chrono::nanoseconds(0)).count();
    const double averageNs = static_cast<double>(sumNs) / static_cast<double>(samples.size());
    const auto p99 = samples[static_cast<std::size_t>(0.99 * static_cast<double>(samples.size()))];
    const auto p999 = samples[static_cast<std::size_t>(0.999 * static_cast<double>(samples.size()))];

    const double periodNs = static_cast<double>(controlLoop.period().count());
    const double cpuDutyPercent = (averageNs / periodNs) * 100.0;
    const double memoryMb = peakResidentMemoryMb();

    // --- Report (docs/architecture.md section 3.13 format) ---
    std::cout << "Robot Emulator Benchmark\n";
    std::cout << "-------------------------\n";
    std::cout << "CPU: " << readCpuModel() << " | OS: " << readOsPrettyName() << " | Compiler: GCC " << __VERSION__
               << " (-std=c++23) | Build: " << ROBOT_BENCHMARK_BUILD_TYPE << "\n";
    std::cout << "Control frequency: 1 kHz\n\n";
    std::cout << "Joints:              " << kJointCount << "\n";
    std::cout << "Sensors:             " << (kJointCount * 4) << "\n\n";
    std::printf("CPU:                 %.1f%%\n", cpuDutyPercent);
    std::printf("Memory:              %.1f MB\n", memoryMb);
    std::printf("Average cycle:       %.2f us\n", averageNs / 1000.0);
    std::printf("P99 cycle:           %.2f us\n", static_cast<double>(p99.count()) / 1000.0);
    std::printf("P99.9 cycle:         %.2f us\n", static_cast<double>(p999.count()) / 1000.0);
    std::cout << "Deadline misses:     0 (unpaced measurement loop — see task brief Non-Goals)\n";

    // Sink is printed so the compiler cannot dead-code-eliminate the sensor
    // reads/checks above as unused work.
    std::cout << "\n[diagnostic] sink=" << sink << " joint0 moved " << positionBefore << " -> " << positionAfter
               << " rad\n";

    return 0;
}
