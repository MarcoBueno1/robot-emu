# Robot Emulator Framework

A high-performance industrial robot controller emulator written in modern C++, with no heavy runtime dependencies.

> **Status:** 🚧 Phases 1–11 implemented, plus the `apps/robot-emulator` server integration (see below). Phase 12 (optional visualization) not yet started. See [Roadmap](#roadmap).
>
> `robotctl` (Phase 7) now works end-to-end against a real running server: `apps/robot-emulator` owns a live `Robot`/`ControllerStateMachine`/`ControlLoop`, dispatches the seven commands `robotctl` sends, and enforces emergency stop and watchdog-triggered safety. See [`server-integration.md`](docs/task-briefs/server-integration.md).

---

## What is this?

Most open-source "robot simulators" solve only the visual part: a 3D mesh moving in response to commands. That's useful for demos, but it doesn't help develop and test **real control software** — which has to deal with state machines, binary protocols, cycle jitter, sensor faults, and safety limits.

**Robot Emulator Framework** takes the inverse approach: it emulates the *behavior of a real robot controller* — its network interface, its state machine, its sensors, its faults — and treats 3D visualization as an optional consumer, not the core of the system.

> High-performance C++ framework that emulates the firmware/controller of an industrial robot — deterministic control loop, custom binary protocol, virtual hardware, fault injection, and automated tests — enabling control software to be developed and validated without physical hardware.

### What this project is
- A **controller behavior** emulator, not a physics engine.
- A library/service that speaks the same "language" (protocol, states, faults) as a real robot.
- A **C++ systems engineering** project: concurrency, soft real-time, binary protocols, low resource footprint.

### What this project is not
- Not a physics simulation engine (no collision, no full rigid-body dynamics).
- Does not depend on ROS, Gazebo, or Webots in the core — those are, at most, optional integrations at the edge.
- Not a GUI. Visualization is a protocol client, not part of the core.

Full rationale in [`docs/architecture.md`](docs/architecture.md).

---

## Quick Start

```bash
mkdir -p build-release && cd build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)

./robot-emulator 9000 &          # starts the server, 6-joint scenario, port 9000
./robotctl 127.0.0.1:9000 status
./robotctl 127.0.0.1:9000 enable
./robotctl 127.0.0.1:9000 move-joint 2 45
./robotctl 127.0.0.1:9000 status
```

---

## Documentation

| Document | Purpose |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | Full technical design: architecture, requirements, protocol, safety, roadmap |
| [`docs/task-briefs/`](docs/task-briefs/) | Self-contained, phase-by-phase implementation briefs — each one is written so it can be handed to a contributor without needing the rest of the project history |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | Coding standards, header policy, and how the phase/task-brief workflow works |
| [Benchmark](#benchmark) | Real, measured performance numbers for the core control-loop/sensor workload |
| [`server-integration.md`](docs/task-briefs/server-integration.md) | How `apps/robot-emulator` wires everything together, including its concurrency design |

---

## Roadmap

Each phase delivers something that compiles, runs, and is testable — no "big bang" at the end.

| Phase | Deliverable | Brief |
|---|---|---|
| 1 | ✅ Core: `Robot` with N `Joint`s (position, velocity, limits) | [`phase-01-core.md`](docs/task-briefs/phase-01-core.md) |
| 2 | ✅ Controller state machine | [`phase-02-controller-state-machine.md`](docs/task-briefs/phase-02-controller-state-machine.md) |
| 3 | ✅ Deterministic real-time control loop (1 kHz) | [`phase-03-control-loop.md`](docs/task-briefs/phase-03-control-loop.md) |
| 4 | ✅ Motion: trajectory, velocity, acceleration | [`phase-04-motion.md`](docs/task-briefs/phase-04-motion.md) |
| 5 | ✅ Virtual hardware: motor, encoder, brake, IO | [`phase-05-virtual-hardware.md`](docs/task-briefs/phase-05-virtual-hardware.md) |
| 6 | ✅ Binary protocol over TCP | [`phase-06-protocol.md`](docs/task-briefs/phase-06-protocol.md) |
| 7 | ✅ `robotctl` CLI | [`phase-07-cli.md`](docs/task-briefs/phase-07-cli.md) |
| 8 | ✅ Safety: limits, E-stop, watchdog | [`phase-08-safety.md`](docs/task-briefs/phase-08-safety.md) |
| 9 | ✅ Sensors: encoder, temperature, current, proximity (force/torque deferred — see brief) | [`phase-09-sensors.md`](docs/task-briefs/phase-09-sensors.md) |
| 10 | ✅ Fault injection registry/dispatcher (not yet wired to real components — see brief) | [`phase-10-fault-injection.md`](docs/task-briefs/phase-10-fault-injection.md) |
| 11 | ✅ Performance optimization (measured) | [`phase-11-benchmark.md`](docs/task-briefs/phase-11-benchmark.md) |
| 12 | Optional visualization | _not written yet_ |
| 13 | *(optional, future)* ROS2 Bridge | _not written yet_ |
| 14 | *(optional, future)* Secure Communication Layer | _not written yet_ |

Static analysis (MISRA C++ subset, `clang-tidy`, `lizard`) runs from Phase 1 onward, across every phase. See `docs/architecture.md` section 3.15.

---

## Benchmark

Real, measured numbers from `apps/robot-benchmark` — a 6-joint, 24-sensor scenario driving `ControlLoop::step()` for 100,000 cycles, following the exact methodology in [`phase-11-benchmark.md`](docs/task-briefs/phase-11-benchmark.md). Deadline misses are 0 by construction: this benchmark measures the workload's own computational cost directly (unpaced), not a real-time-paced background loop — see the brief for why.

```
Robot Emulator Benchmark
-------------------------
CPU: Intel(R) Xeon(R) Processor @ 2.10GHz | OS: Ubuntu 24.04.4 LTS | Compiler: GCC 13.3.0 (-std=c++23) | Build: Release
Control frequency: 1 kHz

Joints:              6
Sensors:             24

CPU:                 0.0%
Memory:              4.2 MB
Average cycle:       0.19 us
P99 cycle:           0.27 us
P99.9 cycle:         0.47 us
Deadline misses:     0 (unpaced measurement loop — see task brief Non-Goals)
```

**Environment caveat:** measured in this repository's actual sandbox/CI-style container (single-core virtualized Intel Xeon, not a dedicated multi-core workstation) — different from `docs/architecture.md` section 3.13's own illustrative example (`AMD Ryzen 7 | Ubuntu 25.10 | GCC 15.2`). Every number above is real and reproducible by running `robot-benchmark` yourself; it is not a representative absolute number for a dedicated deployment target, only a real measurement of this codebase's relative efficiency.

**Conclusion (measured, not guessed):** a 1 kHz control loop has a 1000 µs period per cycle. This scenario's average cycle (~0.19 µs, ~0.02% of that budget) shows the existing design choices made throughout Phases 1–10 — no heap allocation on any hot path, exact analytic solutions instead of iterative approximations (`VirtualMotor`, `TemperatureSensor`), a flat array instead of a hash map for `ControllerStateMachine`'s small transition table — already leave enormous headroom. No optimization was made in this phase: the data didn't support one, and inventing a change without that support would violate section 3.13's own "never estimated" principle. Run `robot-benchmark` yourself (`cmake --build build-release --target robot-benchmark && ./build-release/robot-benchmark`, built with `-DCMAKE_BUILD_TYPE=Release`) to reproduce.

---

## Requirements

- **Language:** C++23
- **Compiler:** GCC 15+ (Ubuntu 25.10 default) or Clang 20+
- **Build system:** CMake ≥ 3.25

---

## License

MIT License — see [`LICENSE`](LICENSE). Anyone may use this project for any purpose, including commercial use, provided the copyright notice is retained.

Copyright (c) 2026 Marco Bueno

---

## Author

**Marco Bueno**
Email: bueno.marco@gmail.com
LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
