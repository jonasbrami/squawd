# Documentation index

The repository contains current operations guides, living project state,
forward-looking design documents, and historical experiment evidence. They do
not all describe the same point in time. Use this index to choose the right one.

## Current sources of truth

Read these in order:

1. [Root README](../README.md) — product scope, prerequisites, supported path,
   known boundaries.
2. [Architecture](architecture.md) — active processes, modules, interfaces,
   data flow, safety model.
3. [Project state](PROJECT-STATE.md) — active goal, last verification, blockers,
   and next bounded step.
4. [Demo runbook](RUN-DEMO.md) — exact local cockpit-demo operations.
5. [Model provisioning](../models/README.md) — local weight artifacts,
   provenance, and licenses.

## Evaluation and benchmark guides

- [Evaluation harness](../evals/README.md) — task schema, truth isolation,
  scripted baselines, and live-run recipe.
- [Capacity benchmark](../bench/README.md) — historical render/simulator scaling.
- [`docs/benchmarks/`](benchmarks/) — dated evidence. A benchmark result proves
  the named revision/configuration, not necessarily the current worktree.

Recent evidence families:

- `m2-*`, `m3*`, `m5-*`: single-drone rebuild perception/fusion/eval gates.
- `demo-*`, `w0-*` through `w5-*`: cockpit demo prototype.
- `deep-perception-m1.md` through `deep-perception-m4.md`: optional host-GPU
  deep-perception sidecar and acceptance limits.
- `EVALS-*`: earlier task/model/swarm experiments.

## Designs and plans

`docs/superpowers/specs/` and `docs/superpowers/plans/` explain why the system
was designed and how milestones were proposed. They are immutable design
records unless explicitly revised. Implementation and current operations take
precedence when a design was superseded.

Key designs:

- `2026-07-18-single-drone-rebuild-design.md`
- `2026-07-19-interface-specification.md`
- `2026-07-28-demo-prototype-design.md`

## Reviews

[`gpt-review/`](../gpt-review/) is a static 2026-08-01 codebase review containing
an accurate single-drone overview, a detailed architecture snapshot, and
prioritized engineering findings. It is an audit, not the living documentation;
revalidate individual findings against the worktree.

Review files embedded under `docs/**/reviews/` are decision evidence for a
specific design or gate. They should not be treated as global instructions.

## Historical scope

Documents dated before the single-drone rebuild may describe:

- a Commander agent;
- N autonomous drone agents;
- `/swarm/*` application topics;
- `agents/swarm/run.py` and `agents/swarm/commander.py`;
- the `run_swarm_demo.sh` end-to-end path.

Those claims are historical unless the missing assembly is restored and tested.
The current application uses one `PilotAgent`, `/pilot/*` topics, and the
single-drone cockpit.

## Maintenance rule

When behavior changes:

- update the root README for supported scope or prerequisites;
- update `architecture.md` for process, module, or interface changes;
- update `RUN-DEMO.md` for operator commands and environment variables;
- update `PROJECT-STATE.md` at a deliberate pause, with a dated verification;
- add benchmark evidence as a new dated file rather than rewriting old results.
