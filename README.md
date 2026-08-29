# Robot Emulator Framework

A high-performance industrial robot controller emulator written in modern C++, with no heavy runtime dependencies.

> **Status:** 🚧 Phases 1–7 implemented (core data model, controller state machine, real-time control loop, trapezoidal trajectory planner, virtual hardware, binary protocol over TCP, `robotctl` CLI). Phase 8 (safety: limits, E-stop, watchdog) not yet started. See [Roadmap](#roadmap).
>
> **Known gap:** `robotctl` has nothing real to connect to yet — `apps/robot-emulator` (the server that would actually own a `Robot`/`ControllerStateMachine`/`ControlLoop` and execute commands) hasn't been built. See [`phase-07-cli.md`](docs/task-briefs/phase-07-cli.md) Non-Goals.

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

## Documentation

| Document | Purpose |
|---|---|
| [`docs/architecture.md`](docs/architecture.md) | Full technical design: architecture, requirements, protocol, safety, roadmap |
| [`docs/task-briefs/`](docs/task-briefs/) | Self-contained, phase-by-phase implementation briefs — each one is written so it can be handed to a contributor without needing the rest of the project history |
| [`CONTRIBUTING.md`](CONTRIBUTING.md) | Coding standards, header policy, and how the phase/task-brief workflow works |

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
| 8 | Safety: limits, E-stop, watchdog | _not written yet_ |
| 9 | Sensors: encoder, temperature, current, force/torque | _not written yet_ |
| 10 | Fault injection | _not written yet_ |
| 11 | Performance optimization (measured) | _not written yet_ |
| 12 | Optional visualization | _not written yet_ |
| 13 | *(optional, future)* ROS2 Bridge | _not written yet_ |
| 14 | *(optional, future)* Secure Communication Layer | _not written yet_ |

Static analysis (MISRA C++ subset, `clang-tidy`, `lizard`) runs from Phase 1 onward, across every phase. See `docs/architecture.md` section 3.15.

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
