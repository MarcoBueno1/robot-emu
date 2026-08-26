<!--
Template for a phase task brief. Copy this file to `phase-NN-<short-name>.md`,
fill in every section, and delete this comment block.

A brief is "done" being written when a competent contributor who has read
ONLY this file plus the referenced docs/architecture.md sections could
implement the phase correctly without asking a clarifying question.
-->

# Phase NN — <Phase Title>

**Status:** Not Started
**Prerequisites:** <list prior briefs that must be `Done`, or "None">

---

## 1. Context

<2-4 sentences of background, self-contained. Point to specific
docs/architecture.md sections (e.g. "section 3.4") for anything that needs
more depth than fits here — do not summarize the whole document.>

## 2. Goal

<One or two sentences: what this phase delivers, stated as an outcome.>

## 3. Non-Goals / Out of Scope

<Explicit list of things that might seem related but are deliberately
deferred to a later phase. Naming the phase they land in prevents scope
creep and prevents the implementer from "helpfully" building ahead.>

- <item> — deferred to Phase <N>

## 4. Inputs

<Exact list of files the implementer needs to read before starting. Nothing
more, nothing implied.>

- `docs/architecture.md`, sections <X.Y>–<X.Z>
- <any prior phase's public headers this phase depends on>

## 5. Deliverables

<Exact file paths to be created or modified. Not a description — a list.>

- `include/robot/<...>`
- `src/<...>`
- `tests/<...>`

## 6. Interfaces / Contracts

<Exact public signatures (class definitions, function signatures, enums)
that this phase must expose, verbatim or close to it. Later phases and
other implementers will code against these — they are not suggestions.>

```cpp
// exact interface here
```

## 7. Acceptance Criteria (Definition of Done)

- [ ] <criterion>
- [ ] <criterion>
- [ ] Every new file carries the standard header (see `CONTRIBUTING.md`)
- [ ] Static analysis passes (`cppcheck --addon=misra`, `clang-tidy`, `lizard`) with no new violations
- [ ] Build succeeds in both Release and Debug, on GCC 15 and Clang 20

## 8. Notes for the Implementer

<Anything a fresh contributor session should know that doesn't fit
elsewhere: known traps, decisions already made and why, what NOT to read
(e.g. "you do not need docs/task-briefs/phase-03-*.md for this phase").>
