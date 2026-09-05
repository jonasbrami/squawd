# GPT codebase review

> **Post-review documentation update (2026-08-08):** the root README,
> `docs/architecture.md`, demo/model/eval guides, and living project state were
> aligned around the single-drone implementation. R9's documentation-coherence
> remediation is therefore addressed in the working tree. This folder remains
> the dated 2026-08-01 audit; revalidate all other findings against current code.

This folder documents the repository as it exists on the `rebuild-single-drone`
branch, not only the product described by the top-level README.

The central conclusion is that this is currently a **single-drone, LLM-piloted
UAV simulation and evaluation system**. It combines PX4 SITL, Gazebo Harmonic,
ROS 2 telemetry, MAVSDK flight control, camera inference, contact tracking, a
browser cockpit, and a Claude/Kimi-backed pilot. The older Commander-led swarm
is still visible in names, historical documentation, evaluation data, and one
legacy class, but its assembler and Commander implementation are absent.

## Documents

- [PROJECT-OVERVIEW.md](PROJECT-OVERVIEW.md) explains the product, its actual
  current scope, its major capabilities, and how it is intended to be operated.
- [ARCHITECTURE.md](ARCHITECTURE.md) describes the construction, processes,
  modules, interfaces, data flow, coordinate frames, safety layers, and
  evaluation architecture.
- [ISSUES.md](ISSUES.md) contains prioritized findings, evidence, impact, and
  suggested remediation.

## Executive assessment

The codebase has unusually strong seams for a research prototype. Flight logic
is separated from LLM SDK message types, vision backends sit behind protocols,
ground truth is deliberately isolated from the production perception lane, and
the evaluation oracle is deterministic rather than LLM-judged. The single-drone
rebuild is also backed by a broad unit-test inventory and extensive experiment
records.

The main weakness is **state coherence**. The source tree is a single-drone
rebuild while the public README, architecture document, package naming, and
swarm launcher still describe an earlier product. There are also material
safety and deployment gaps: several envelope validators are not connected to
the corresponding tool calls, arbitrary model-authored Python can execute in
the pilot process, the cockpit control surface has no authentication, and a
clean clone lacks the ignored, prebuilt PX4 tree required by the launch scripts.

## Review scope and snapshot

- Static review date: 2026-08-01 (Asia/Dubai).
- Branch observed: `rebuild-single-drone`.
- Base commit observed: `484eee8ac613cb5590211faec0c3eadd4935b3c6`.
- The working tree was intentionally dirty and another agent was editing it
  during review. Uncommitted M6/backend, tracking, eval, test, and benchmark
  changes were treated as part of the current state.
- No source files or existing documentation were changed by this reviewer.
  Only this `gpt-review/` folder was created.
- This is a static code and structure review. Runtime simulation, Docker build,
  live flight behavior, model accuracy, and published benchmark claims were not
  independently revalidated for this deliverable.
