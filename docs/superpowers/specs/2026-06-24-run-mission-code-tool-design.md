# `run_mission` — drone-Claude authors its own MAVSDK mission code

**Date:** 2026-06-24
**Status:** Design approved (revised after spec review), pending final spec review

## Problem

Today each drone-Claude pilots through 8 thin MAVSDK wrapper tools
(`take_off`, `fly`, `goto`, `orbit`, `hover`, `set_speed`, `face`, `land` in
`agents/flight/tools.py`). They only expose MAVSDK's `action` plugin, so the
drone can only approximate a desired trajectory by chaining blocking `goto`
calls — stop-and-go legs, one LLM round-trip per waypoint, no smooth path and no
velocity continuity.

We want to move toward letting drone-Claude **write its own MAVSDK code**
instead of being limited to fixed primitives. The first, contained step is
mission support: a tool through which Claude authors a full async MAVSDK
mission (build MissionItems → upload → arm → start → monitor mission_progress),
exec'd in-process against the live connection.

## Goal

Add one MCP tool, `run_mission(code: str, timeout=None)`, that runs
Claude-authored async MAVSDK Python in the drone process and feeds the result (or
traceback) back to Claude so it can iterate. Prove the "Claude writes MAVSDK code" execution path
on the one maneuver we care about (missions / trajectories) before generalizing.

## Guiding principle for helpers

Injected helpers exist **only to cut repetition and tokens, never to limit
behavior.** Every helper must be (a) ignorable — Claude can write the raw MAVSDK
equivalent instead — and (b) non-hiding — it exposes the SDK's own names/values,
adds no policy, clamps nothing. A helper that would restrict what Claude can fly
(e.g. an altitude cap) is out of scope here.

## Non-goals

- Not removing the existing 8 flight primitives. They stay as a fallback this
  step; cleanup/simplification comes later once the code path proves out.
- Not generalizing to arbitrary MAVSDK beyond missions yet (no offboard
  setpoint streaming, no full `run_mavsdk`). Mission-only first.
- Not sandboxing the executed code. Decision: in-process `exec` sharing the live
  System.
- No behavior-limiting safety in the harness (no altitude/velocity clamp). PX4's
  geofence stays as the (currently warning-only) net; risk accepted for sim — see
  Safety.
- No changes to `look` / `scan` / `report` (non-MAVSDK sensing + comms tools).

## Decisions (from brainstorming + spec review)

| Question | Decision |
|---|---|
| Execution model | In-process `exec()` of an async body in the drone process, sharing the live MAVSDK connection. No sandbox. |
| Scope of first step | Mission-only `run_mission` tool, **added alongside** existing primitives. |
| Snippet contract | **Full lifecycle** — Claude writes build-items → upload → arm → start → monitor itself. Harness only exec's and captures result/errors. |
| Imports | Claude writes its own imports (`from mavsdk.mission import MissionItem, MissionPlan`, etc.). |
| Injected namespace | What Claude can't import or shouldn't repeat: `drone` (live `System`), `mission_item(**kw)` (nan/NONE-defaulting `MissionItem` passthrough), `world_to_geo(east, north, up)` (ENU→GPS), `log(msg)` (capture). |
| Loop blocking (S1) | **Accepted.** A tasked drone is uninterruptible mid-mission, bounded by the mission timeout. Documented limitation; acceptable for sim where the Commander tasks one drone at a time. |
| Timeout (Claude-set) | `run_mission(code, timeout=None)` — **the drone sets the seconds** it expects the mission to need. If omitted, `DEFAULT_MISSION_TIMEOUT_S` (~180 s) applies. No hard ceiling (non-limiting), but a default always applies so it is never unbounded. |
| Timeout behavior (S2) | On timeout/cancel the harness halts the vehicle (`pause_mission`/`hold`) before returning the error, and tells Claude the mission was stopped. |
| Output capture (S3) | Inject a `log(msg)` helper (per-call buffer); **no** process-global `redirect_stdout`. |
| Altitude clamp (S4) | **Not added** — would limit behavior. Risk documented instead. |

## Design

### The tool

`run_mission(code: str, timeout: number = None)` — one MCP tool registered in
`make_drone_options`, added to `tools=[...]` and `allowed_tools=[...]` next to
the existing primitives. `code` is the async body Claude wrote; `timeout` is the
seconds Claude allots the mission (defaults to `DEFAULT_MISSION_TIMEOUT_S` when
unset — no hard ceiling).

### Execution harness

A new async helper (in `agents/flight/ops.py`, e.g. `FlightOps.run_mission`)
wraps the code as the body of an `async def`, compiles and exec's it with the
injected namespace, and awaits it on the drone's running event loop:

```python
src = "async def _snippet():\n" + textwrap.indent(code, "    ")
logs = []
ns = {
    "drone": self.drone,
    "mission_item": _mission_item,          # nan/NONE-defaulting passthrough
    "world_to_geo": self._world_to_geo,     # ENU→GPS, bound to this drone
    "log": logs.append,
}
t = timeout if timeout is not None else DEFAULT_MISSION_TIMEOUT_S   # Claude-set
try:
    exec(compile(src, "<mission>", "exec"), ns)          # compile errors caught here
    ret = await asyncio.wait_for(ns["_snippet"](), timeout=t)
except asyncio.TimeoutError:
    await _halt()                                         # pause_mission / hold (S2)
    return error(f"mission timed out after {t}s; vehicle halted", logs)
except Exception:
    return error(traceback.format_exc(), logs)           # full traceback back to Claude
return ok(ret, logs)
```

- Wrapping in `async def` lets Claude use `await` and top-level `return`.
- `exec` uses a fresh globals dict seeded with the injected names; Python's
  `import` works inside exec, so Claude's own imports resolve normally.
- Closures: `_snippet` is created with `ns` as its globals, so `drone`,
  `mission_item`, `world_to_geo`, `log` resolve as globals inside the snippet.

### Injected helpers (repetition-cutters, non-limiting)

**`mission_item(**kw)`** — a thin passthrough that defaults every `MissionItem`
field to `float('nan')` (and the two enum fields to `…NONE`), then overrides
with whatever Claude passes, using the SDK's **own field names**:

```python
def _mission_item(**kw):
    nan = float("nan")
    d = dict(latitude_deg=nan, longitude_deg=nan, relative_altitude_m=nan,
             speed_m_s=nan, is_fly_through=True,
             gimbal_pitch_deg=nan, gimbal_yaw_deg=nan,
             camera_action=MissionItem.CameraAction.NONE,
             loiter_time_s=nan, camera_photo_interval_s=nan,
             acceptance_radius_m=nan, yaw_deg=nan, camera_photo_distance_m=nan,
             vehicle_action=MissionItem.VehicleAction.NONE)
    return MissionItem(**{**d, **kw})
```

It hides nothing (real field names, every field overridable) and is ignorable
(Claude may call `MissionItem(...)` directly). It removes only the
no-defaults / `nan`-sentinel / enum-path repetition.

**`world_to_geo(east, north, up)`** — the existing `FlightOps._world_to_geo`,
returning `GeoPoint(latitude_deg, longitude_deg, absolute_altitude_m)` for a
world ENU point relative to the drone's live fix. Used for the lat/lon of each
waypoint. (Stateful — bound to the live drone — so not importable.)

**`log(msg)`** — appends to the per-call buffer returned alongside the result;
replaces process-global stdout capture so concurrent components don't cross-leak.

### Coordinates / altitude (fixes review B2)

`world_to_geo` returns **AMSL** `absolute_altitude_m`; `MissionItem`'s
`relative_altitude_m` is **relative to home**. These are NOT interchangeable.
Correct usage, stated in the prompt and the spec: take **lat/lon only** from
`world_to_geo`, and set `relative_altitude_m` to the world `up` (height above
home). The AMSL field is never fed into a MissionItem.

```python
geo = await world_to_geo(east=40, north=40, up=15)
item = mission_item(latitude_deg=geo.latitude_deg,
                    longitude_deg=geo.longitude_deg,
                    relative_altitude_m=15, speed_m_s=5)
```

### Result feedback (the iteration loop)

- Success: return the snippet's return value (or "completed, no return value"
  when `None`) plus the collected `log()` lines, as the tool's text result.
- Any exception (incl. compile-time `SyntaxError`/`IndentationError`): return the
  formatted **traceback** with `is_error: True`, plus any logs so far, so Claude
  sees exactly what failed and rewrites the snippet.

### Timeout / cancellation (fixes review S2)

The timeout is **Claude-set per call** (`run_mission(..., timeout=N)`) — the drone
knows its mission's expected duration. When omitted, `DEFAULT_MISSION_TIMEOUT_S`
(~180 s) applies; there is no hard ceiling (capping would limit behavior), but a
default always applies so the wait is never unbounded — satisfying the "always cap
long waits" rule. On `TimeoutError` (or `CancelledError`), the harness actively
halts the vehicle (`await drone.mission.pause_mission()`, falling back to
`drone.action.hold()`) before returning, because cancelling the Python coroutine
does NOT stop PX4 from flying the already-uploaded mission. The error result
states the mission was halted so Claude doesn't assume a clean failure and stack a
second mission on top.

### Loop blocking (accepted, review S1)

The snippet is awaited inside `DroneAgent.run()`'s single command loop
(drone.py:62-74), so while a monitored mission runs (potentially minutes) the
drone does not pull new `/swarm/cmd` messages. This is accepted for now and
documented: a tasked drone is uninterruptible until its mission finishes or its
(Claude-set) timeout fires. Revisit (cancellable background task) if/when the
Commander needs mid-mission redirects.

### Safety (review S4 — accepted risk)

- No exec sandbox (explicit decision).
- No harness altitude/velocity clamp (would limit behavior, against the guiding
  principle).
- PX4 geofence unchanged and **warning-only**: `GF_MAX_HOR_DIST=300`,
  `GF_MAX_VER_DIST=80`, `GF_ACTION=1` (drone.py:54-59). With arbitrary
  Claude-authored missions this does not *contain* the drone, only warns. Risk
  accepted for the sim demo; raising `GF_ACTION` to an enforcing action is a
  separate future change, not part of this feature.

### System prompt

Add a short block to the drone system prompt in `make_drone_options`:

- You have `run_mission(code, timeout=…)` for trajectories/missions; prefer it
  over chaining `goto` when you want a multi-leg or smooth path. Set `timeout` to
  the seconds you expect the mission to take (you are uninterruptible until it
  finishes or the timeout fires, which halts you).
- Write a full **async** MAVSDK body. Pre-bound (no import needed): `drone` (the
  connected System), `mission_item(**fields)`, `await world_to_geo(east, north,
  up)`, `log(msg)`. Import MAVSDK classes you use (`MissionPlan`, etc.) yourself.
- Coordinates: use `scan`'s world east/north/up; get lat/lon from
  `await world_to_geo(...)`; set `relative_altitude_m` to the world `up` (NOT the
  helper's absolute altitude).
- Skeleton: build items with `mission_item(...)` (use `is_fly_through=True` for a
  smooth path) → `MissionPlan(items)` → `upload_mission` → `arm` →
  `start_mission` → `async for p in drone.mission.mission_progress()` until
  `p.current == p.total`. Remember to `await` everything; `log(...)` anything you
  want reported back.
- Assume you are already airborne when tasked (the Commander tasks you after
  take-off); include a takeoff/`vehicle_action` only if you are on the ground.
- One worked end-to-end example (full 4-line lifecycle) included verbatim.

## Files touched

- `agents/flight/ops.py` — add the `run_mission` exec helper, the `_mission_item`
  factory, the `_halt` helper, the `DEFAULT_MISSION_TIMEOUT_S` constant; reuse the
  existing `_world_to_geo`.
- `agents/flight/tools.py` — register the `run_mission` tool wrapper; add to
  `tools` + `allowed_tools`; add the prompt block (with the worked example).

## Testing

- Unit: the exec harness round-trips a trivial snippet (return value + `log`
  lines captured); an exception returns a traceback with `is_error`; a
  syntax-error snippet returns a compile error, not a crash; `_mission_item()`
  with no args yields a valid all-`nan`/`NONE` MissionItem and overrides apply.
- Integration (sim): task a drone with a multi-waypoint mission; confirm it
  authors a `run_mission` snippet, the drone flies the waypoints, and the result
  feeds back. Confirm a deliberately broken snippet's traceback returns and Claude
  can retry. Confirm timeout halts the vehicle. Bound the sim run with a hard
  deadline.

## Future (out of scope here)

- Generalize to `run_mavsdk` for arbitrary MAVSDK once mission proves out.
- Then retire/collapse the 8 primitives into the code path.
- Cancellable mid-mission (background task) if the Commander needs redirects.
- Offboard setpoint streaming for truly arbitrary (curved/velocity-profiled)
  trajectories.
- Enforcing geofence (`GF_ACTION`) once demos can tolerate it.
