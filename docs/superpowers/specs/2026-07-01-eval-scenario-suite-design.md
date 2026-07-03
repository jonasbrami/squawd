# Eval Scenario Suite — Design

**Date:** 2026-07-01
**Branch context:** builds on the validated `evals/` single-drone harness (branch `bench/swarm-capacity`)
**Status:** approved design, pre-plan

## Goal

Expand the eval harness's task set from one anchor scenario into a graduated **scenario
suite** that makes the harness discriminate between Claude tiers and surfaces where
tools/prompting break down. Feeds the latency-vs-correctness Pareto story and the
"improve tooling & prompting" goal.

## Decisions (locked during brainstorming)

- **Purpose:** a mix, layered — one-variable **ladders** per axis to find each tier's
  failure knee, PLUS compound **capstone** missions that stack axes.
- **Axes (all four):** plan-depth, spatial, ambiguity, obstacle/route. Coordination stays
  out of scope (needs the deferred Commander/swarm layer).
- **Structure:** ladders + capstones. A shared anchor; each axis ladder perturbs only its
  own variable from that anchor; capstones combine axes.
- **Grading:** sim-state oracle only (unchanged). Add five new checks; keep every check a
  pure function over `WorldTrack`.
- **Worlds:** flat `default` for plan-depth/spatial/ambiguity/`c1`; `city` for
  obstacle + `c2`. `baylands` is banned for evals (terrain offsets PX4's local-altitude
  frame ~570 m — see the harness README).

## New oracle checks (`evals/oracle.py`)

Same pattern as the existing `reached`/`coverage`/`alive`/`within_step_budget`: a pure
`fn(track, params, run_meta) -> CheckResult`, registered in `CHECKS`.

| check | grades | params |
|---|---|---|
| `visited_all` | drone passed within `tol_m` of **every** target in `targets` (any order) | `targets, tol_m` |
| `ordering` | those targets were first-reached in the **given sequence** (strictly increasing first-reach times) | `sequence, tol_m` |
| `altitude` | at closest approach to `target` (or over the whole track if no target), altitude stayed within `[min_m, max_m]` | `target?, min_m, max_m` |
| `clearance` | min distance from the drone to any building footprint stayed `≥ margin_m` | `margin_m` |
| `dwell` | drone held within `tol_m` of `target` for a continuous span `≥ hold_s` | `target, tol_m, hold_s` |

`ordering` supersedes `visited_all` (sequence ⊃ set); both exist because patrol-any-order
and strict-route are distinct signals.

## Plumbing (small)

- `WorldTrack` gains `buildings: list[dict]` (name + footprint box), populated once by the
  sampler from `World.buildings`, so `clearance` can grade. Empty on worlds without
  buildings (flat) — `clearance` then trivially passes, and no flat task uses it.
- `Snapshot` already carries `t` (for `ordering`/`dwell` timing) and `DronePose.alt` (for
  `altitude`). No other data changes.
- `ordering`/`dwell` depend on the **settle phase** already in `run_cell` so late arrivals
  and holds are actually present in the sampled track.

## Scenario suite

Layout — subdir per axis, one file per rung; `difficulty` metadata tags the rung so the
report can pivot the knee:

```
evals/tasks/{anchor,plan_depth,spatial,ambiguity,obstacle,capstone}/*.yaml
```

**Anchor** (`default`): the existing validated go-to-one-marker task (reused as the
baseline all ladders perturb from).

**Plan-depth ladder** (`default`, `ordering`; distance/tol held constant):
`p1` 2 ordered waypoints → `p2` 3 → `p3` 4 (patrol) → `p4` 4 + return-to-home (5 legs).
`within_step_budget` scales with legs.

**Spatial ladder** (`default`, `reached`/`altitude`; single reach):
`s1` 60 m → `s2` 130 m → `s3` 250 m (pure distance, tol held at 10 m) →
`s4` reach + end in altitude band [18,22] m (3-D precision).

**Ambiguity ladder** (`default`, spec decreases; `coverage`/loose `reached`):
`am1` explicit coords (≈anchor) → `am2` "≈120 m north-east, then stop" (graded vs the
implied point, loose tol) → `am3` "search the NE quadrant" (`coverage ≥70%`, no target) →
`am4` "sweep the area, finish at the far corner" (`coverage` + loose `reached`).

**Obstacle ladder** (`city`, `clearance`+`reached`+`alive`; gated — see below):
`o1` target behind 1 building → `o2` behind a 2–3 building cluster → `o3` across a denser
field. `clearance margin ≥ 5 m`.

**Capstones** (compound):
- `c1` recon patrol (`default`): 3 ordered waypoints at a set altitude band, `dwell 8 s`
  at the last → `ordering`+`altitude`+`dwell`+`alive`.
- `c2` obstacle patrol (`city`): 2 ordered waypoints either side of a cluster, hold
  clearance, end in tol → `ordering`+`clearance`+`reached`+`alive`.

~15–18 scenarios. At K=5 × 3 tiers this is substantial sim time, so the operator subsets
via `--tasks` and runs the flat suite and the city suite against their respective sims;
results append into one merged report.

## Report knee-view (`evals/report.py`)

Add `render_ladders()`: for each axis, pivot **success-rate by rung × tier** so the knee
is readable directly, e.g.

```
## Spatial ladder
rung        opus   sonnet  haiku
s1 (60m)    100%   100%    100%
s2 (130m)   100%   100%     60%
s3 (250m)   100%    80%      0%
```

Reads the `difficulty` tag + `assignment` already present in each result row — no new
data, just a pivot. The existing per-cell success/latency table is unchanged; latency
stays there.

## World orchestration

The harness runs against whichever sim is up; `setup.world` documents intent but does not
launch. A full sweep is two passes: the flat suite against a `default` container, and
obstacle + `c2` against a `city` container. Both write to the same out-dir (append/resume)
so one merged `RESULTS.md` covers everything. The README gains a `city` bring-up variant.

## City-world gate

Obstacle rungs are the one real risk. Before authoring them, validate `city` the way we
validated flat: (1) a grounded drone reads `z ≈ 0` (no baylands-style altitude offset);
(2) `city_boxes.json` loads and `World.buildings` returns boxes with usable footprints for
`clearance`. If city fails (1), obstacle + `c2` are **held** (documented, not silently
broken) and the suite ships plan-depth/spatial/ambiguity/`c1`, which need only the already
validated flat world.

## Testing

- Every new oracle check is TDD'd pure against hand-built `WorldTrack`s, like the existing
  four (no sim needed).
- The sampler's building capture is unit-tested with a fake `World`.
- Each authored task YAML must load through `spec.load_task` (validates checks + fields).
- One `city`-gated live smoke of an `o1` cell confirms `clearance` end-to-end, mirroring
  the flat smoke already run.

## Out of scope

- Coordination axis / multi-drone (needs the deferred Commander/swarm layer).
- Physical Gazebo marker spawning; targets remain spec-declared coordinates.
- Camera-perception grading (still sim-oracle only; `look`/`scan` are tools the agent may
  use, but success is graded on spatial outcome).
- A YAML generator for rungs — files are hand-authored for now; revisit if the suite grows.
