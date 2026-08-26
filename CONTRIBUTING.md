# Contributing

This project is developed in well-delimited, incrementally shippable phases. Each phase has a **task brief** in [`docs/task-briefs/`](docs/task-briefs/) — a self-contained specification meant to be handed to a contributor without requiring any other context than the brief itself plus the referenced sections of [`docs/architecture.md`](docs/architecture.md).

## Why task briefs exist

The goal is to let different phases be implemented independently — potentially by different people — while still producing a codebase that fits together correctly the first time. A task brief is not a summary of a conversation; it's a standalone contract: context, exact interfaces, deliverables, and an explicit Definition of Done. If a brief is ambiguous enough that two different implementers could reasonably build incompatible things, the brief has a bug and should be fixed before implementation starts.

## Workflow

1. Pick the next `Not Started` brief from the [Roadmap table in README.md](README.md#roadmap) (phases are meant to be done in order — later phases depend on the public interfaces earlier phases establish).
2. Read the brief in full, plus any `docs/architecture.md` sections it references. Do not start from assumptions about the rest of the project — the brief lists everything you need.
3. Implement exactly the deliverables and interfaces listed in the brief. If something outside the brief's scope seems necessary, stop and flag it rather than silently expanding scope.
4. Verify every item in the brief's **Acceptance Criteria** checklist before considering the phase done.
5. Open a pull request referencing the brief.

## Coding standards

- **Language:** C++23, no compiler extensions (`-std=c++23`, not `gnu++23`).
- **Errors as values:** use `std::expected<T, E>` for recoverable errors. No exceptions on the control path (see `docs/architecture.md` section 3.15).
- **No dynamic allocation** on the control path (`Joint::update`, `Robot::update`, and anything called from the real-time loop once Phase 3 exists).
- **`const`-correctness and `noexcept`** applied explicitly wherever the operation cannot fail via exception.
- Functions stay small, single-responsibility, with bounded cyclomatic complexity (CI enforces CCN ≤ 15 via `lizard`).
- Static analysis must pass: `cppcheck --addon=misra`, `clang-tidy`, `lizard`. See `scripts/static-analysis.sh` once it exists.

## Mandatory file header

Every `.hpp`, `.cpp`, `.cmake`, `CMakeLists.txt`, and script (`.sh`, `.py`) must start with:

```cpp
// -----------------------------------------------------------------------------
// Robot Emulator Framework
// -----------------------------------------------------------------------------
// Author:   Marco Bueno
// Email:    bueno.marco@gmail.com
// LinkedIn: https://www.linkedin.com/in/marco-bueno-44417981/
// License:  MIT (see LICENSE file at project root)
// -----------------------------------------------------------------------------
```

(use `#` instead of `//` in shell/Python scripts)

This applies regardless of who or what wrote the file — contributor `scripts/check-headers.sh` (once it exists) enforces this in CI.

## Language policy

All documentation, comments, identifiers, and commit messages in this repository are written in **English**, with no exceptions.

## License

By contributing, you agree your contribution is licensed under the project's MIT License (see [`LICENSE`](LICENSE)).
