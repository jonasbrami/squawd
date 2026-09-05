# Swarm coordination evals (C0 operator / C1 commander) — design

**Date:** 2026-07-04 · **Status:** approved in design session
**Goal:** extend the eval harness from single-drone to multi-drone so we can
measure coordination (allocation, deconfliction, cooperative dynamics, timing),
fleet-size scaling (N=2→8), the delegation tax (commander vs operator on
identical tasks), and per-role tier mixes (commander=opus;drones=haiku).

Builds on: single-drone harness (evals/), dynamic-scenarios suite
(2026-07-03 spec), the swarm stack (Commander/DroneAgent), and the original
three-layer intent (single_drone → commander → swarm) from the 2026-06-29 spec.

## Decisions (from the session)

1. **Ladder of two rungs.** C0 "operator": ONE LLM client controls all N
   drones directly. C1 "commander": the production architecture — a Commander
   client dispatching autonomous per-drone LLM agents. C0 first: it isolates
   pure coordination reasoning, is cheap to build, and calibrates C1.
2. **All four task axes in the first suite:** allocation, deconfliction,
   cooperative dynamics, coordinated timing.
3. **C1 sensing:** Commander gets the live situation map (situation_text) +
   drone reports — generous sensing keeps failures attributable to reasoning.
4. **Fleet size:** start at N=2 and scale to 8 (w7 capstone at N=2/4/8);
   N itself is a measured axis.

## C0 operator

- One ClaudeSDKClient; tools = the existing per-drone MCP namespaces
  (mcp__d0__*, mcp__d1__*, …) built by make_drone_options' tool factory, N×.
- **New primitive `goto_all(moves=[{drone, east, north, up}, ...])`** —
  issues all moves concurrently, returns when ALL arrive (per-drone results).
  Rationale: sequential blocking gotos serialize the fleet — the harness, not
  the model, would forbid coordination (the blocking-goto lesson at fleet
  granularity). Per-drone wait=false remains for advanced interleaving.
- System prompt: fleet framing ("you fly ALL of these drones"), PLAN paragraph
  extended with "assign drones to goals explicitly before moving".

## C1 commander (phase 2 of the build)

- CommanderClient: tools = dispatch(drone_id, task_text), situation (live
  fused map), plus incoming report messages injected between turns; each
  drone is an unchanged single-drone agent (own client, own tier).
- **Same task YAMLs as C0** (target_layer selects the harness path): C1 − C0
  on identical tasks = the delegation tax.
- Per-role tiers via existing assignments syntax: commander=opus;drones=haiku.
- Infra: concurrent clients need per-agent CLAUDE_CONFIG_DIR (known >4-agent
  ~/.claude.json corruption trap; solved pattern in the swarm stack).

## Task ladder

| rung | axis | N | sketch | oracle |
|---|---|---|---|---|
| w1_split_reach | entry | 2 | 2 far-apart targets; wall clock too short for one drone to visit both | targets_covered + budgets |
| w2_allocation | allocation | 2 | asymmetric cluster; fleet path budget between optimal and worst assignment (machine-verified) | targets_covered + path_length (fleet) |
| w3_crossing | deconfliction | 2 | naive straight lines cross mid-field simultaneously; altitude layering is the legal dodge | reached ×2 + fleet_separation ≥ 8 m |
| w4_double_intercept | coop dynamic | 2 | tag courier (N edge) AND rover (SE plaza) within 90 s of each other — solo impossible | intercept ×2 + within_window |
| w5_sync_mark | timing | 2 | drone A over mark A while drone B over mark B, same 5 s window | simultaneous |
| w6_pincer_relay | coop dynamic hard | 2 | region-handoff shadow (deferred to rung 2, calibrated by w1-w5 results) | dwell + handoff |
| w7_fleet_survey | N-scaling capstone | 2/4/8 | N zones, N drones, fleet budget + separation | coverage + fleet_separation + budgets |

Anti-luck: every rung ships pilot (must-PASS, scripted fleet flight) +
null_pilot (must-FAIL: w2 worst assignment blows the budget; w3 same-altitude
straight lines violate separation; w4 solo attempt misses the window).

## Harness deltas

- **FleetHarness**: owns N MAVSDK links + telemetry subs (N=1 degenerates to
  today's DroneHarness); operator client builder wires N FlightOps + goto_all.
- Lift runner's n_drones==1 gate keyed on target_layer ∈ {operator, commander}.
- New oracle checks (pure, over Snapshot.poses which is already N-aware):
  - targets_covered: every listed target visited by SOME drone (tol_m)
  - fleet_separation: min pairwise own-drone 2D distance ≥ margin (grace_s
    for spawn adjacency; drones spawn 3 m apart)
  - simultaneous: ∃ snapshot where drone i within tol of A AND drone j within
    tol of B (assignment-free: any drone-to-mark matching counts)
  - within_window: two named events (intercepts/reaches) occur ≤ window_s apart
- Pilot: script steps gain optional `drone:` field (default 0); behaviors get
  the FlightOps list. Dual-baseline gate unchanged.
- Reset: soft_reset/check_home/ferry are already N-aware; validate at N=2,
  stagger ferry altitudes if simultaneous ferries conflict.
- Sampler/scan/report: already N-aware (poses dict, drone contacts in scan).
- Sim: evals-fleet containers, SWARM_N per config (2 default; 4/8 for w7);
  flat world for w1-w3/w5/w7, dynamic world for w4/w6. Known trap: PX4
  instance-0 spawn race at higher N (restart px4 -i 0 if N-1 drones come up).
- Budgets: sized WITH the observation/ToolSearch tax lesson from the dynamic
  suite (pilots don't pay it; LLMs do).

## Experiments

- E1: C0 screening — w1-w5, N=2, {opus, sonnet, haiku}, K=2.
- E2: N-scaling — w7 at N=2/4/8, best tier + haiku.
- E3: delegation tax — C1 vs C0 on identical tasks, same tier.
- E4: role-mix matrix — commander=opus;drones=haiku vs all-opus vs all-haiku
  vs commander=haiku;drones=opus (cost/quality frontier).

## Build order

1. Harness deltas (FleetHarness, goto_all, oracle checks, pilot drone field)
2. w1-w3 + pilot gates on an N=2 flat container
3. w4/w5 (+N=2 dynamic container) → E1 screening
4. w7 + N=4/8 containers → E2 scaling
5. C1 commander path → E3/E4
