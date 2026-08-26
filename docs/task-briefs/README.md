# Task Briefs

Each file in this directory is a **self-contained implementation spec** for one phase of the roadmap (see `README.md` at the repository root). "Self-contained" means: a brief plus `docs/architecture.md` should be enough for a competent C++ contributor — human or LLM — to implement the phase correctly, without needing any other conversation history or prior context.

## Why this exists

Different phases of this project may be implemented by different people, or by different LLM sessions/models entirely. Without a formal handoff artifact, context gets lost between sessions and quality drifts — interfaces get reinvented slightly differently each time, error-handling conventions slip, and "obvious" project-wide decisions (C++23, `std::expected` instead of exceptions, no dynamic allocation on the control path, MIT license with the standard file header) get forgotten or reinterpreted. A task brief exists to make those decisions explicit, once, per phase — not something to be inferred from a chat transcript.

## Structure every brief follows

See [`TEMPLATE.md`](TEMPLATE.md) for the exact section layout. In short, every brief has:

- **Status** — `Not Started`, `In Progress`, or `Done`.
- **Prerequisites** — which other briefs must be complete first.
- **Context** — the minimum background needed, in a few sentences, with pointers to the exact `docs/architecture.md` sections for anything deeper.
- **Goal / Non-Goals** — what this phase delivers, and just as importantly, what it deliberately does not.
- **Inputs** — exactly which files/docs to read before starting.
- **Deliverables** — exact file paths to be created or modified.
- **Interfaces / Contracts** — exact public signatures other phases will depend on. These are not suggestions; changing a contract defined here requires updating this brief and flagging the change, because later phases are written assuming it holds.
- **Acceptance Criteria** — a checklist. The phase is not done until every item is checked.
- **Notes for the implementer** — anything an LLM picking this up cold should know, including an explicit statement of what it does *not* need to read.

## If you are an LLM implementing a brief

Read only:
1. This file.
2. The specific brief you're implementing.
3. The `docs/architecture.md` sections the brief points you to (not the whole document, unless the brief says so).
4. `CONTRIBUTING.md` for coding standards and the mandatory file header.

You do not need any other file in this repository unless the brief's **Inputs** section says so. If the brief seems to require information it doesn't provide, that's a defect in the brief — stop and flag it rather than guessing.
