# Track Primitive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give drones a classical real-time pursuit controller (`track` / `track_all`) the LLM parameterizes but does not fly, then measure how much of the dynamic-task ceiling it removes.

**Architecture:** Pure guidance/estimation logic in `agents/flight/track.py` (unit-testable, no MAVSDK/gz imports); the control loop as `FlightOps.track` streaming PX4 offboard position+velocity setpoints at 10 Hz; MCP tools `track` (per drone) and `track_all` (fleet-concurrent). Design: `docs/superpowers/specs/2026-07-06-track-primitive-design.md`.

**Tech Stack:** Python asyncio, MAVSDK offboard plugin, gz-transport pose feed (GzPoses), existing evals harness.

## Global Constraints

- Tasks/oracles/budgets in eval YAMLs are FROZEN except the `pilot:` blocks named in Task 4 — never touch oracle checks, tolerances, prompts, or budgets.
- `null_pilot:` blocks are FROZEN (they must keep failing).
- Controller constants: CTRL_HZ=10.0, MAX_DURATION_S=120.0, MAX_SPEED_MPS=12.0, V_EMA_ALPHA=0.35.
- All distances the controller logs/reports are HORIZONTAL (east/north), matching the oracle's `_mover_sep`.
- No new dependencies.
- Live-sim steps only run inside the eval containers (evals-dyn, evals-fleetdyn); nothing ROS/gz/MAVSDK imports at module top level outside `__init__`-style guards already used in the repo.

---

### Task 1: Guidance core (`agents/flight/track.py`) + unit tests

**Files:**
- Create: `agents/flight/track.py`
- Create: `tests/test_track.py`

**Interfaces:**
- Produces: `TargetEstimator` (`.update(t, e, n)`, `.ve`, `.vn`, `.ready`, `.speed()`), `intercept_t_go(r_e, r_n, v_e, v_n, s) -> float|None`, `control_ref(mode, me_e, me_n, tgt_e, tgt_n, est, speed, standoff_e=0.0, standoff_n=0.0) -> (ref_e, ref_n, ff_ve, ff_vn)`, `clamp_ref_alt(world, ref_e, ref_n, alt) -> float`, `TrackLog(within_m)` (`.sample(t, gap)`, `.min_gap`, `.mean_gap()`, `.best_dwell`, `.n`), constants `CTRL_HZ`, `MAX_DURATION_S`, `MAX_SPEED_MPS`, `V_EMA_ALPHA`.
- Consumes: nothing from this repo (pure module).

- [ ] **Step 1: Write `agents/flight/track.py` exactly:**

```python
"""Classical real-time tracking: the pure guidance/estimation logic behind
FlightOps.track. The LLM sets the WHAT (target, mode, altitude, duration,
speed cap); this module computes the per-tick control reference and
FlightOps streams it to PX4 offboard as position + velocity feedforward, so
PX4's own cascade (v_des = v_ff + MPC_XY_P*(p_sp - p)) is the PD law — see
docs/superpowers/specs/2026-07-06-track-primitive-design.md.

No MAVSDK/gz imports: everything is state-in/state-out so the controller is
unit-testable without a sim."""
import math

CTRL_HZ = 10.0
MAX_DURATION_S = 120.0
MAX_SPEED_MPS = 12.0
V_EMA_ALPHA = 0.35     # EMA weight of the newest finite-difference sample


class TargetEstimator:
    """Velocity from finite differences of (sim_t, e, n) samples, EMA-smoothed.
    A repeated stamp (stale gz sample between our ticks) is skipped, never
    treated as zero velocity."""

    def __init__(self) -> None:
        self._last = None                    # (t, e, n)
        self.ve = 0.0
        self.vn = 0.0
        self.ready = False                   # True once one real difference seen

    def update(self, t: float, e: float, n: float) -> None:
        if self._last is None:
            self._last = (t, e, n)
            return
        dt = t - self._last[0]
        if dt <= 1e-3:
            return
        ve = (e - self._last[1]) / dt
        vn = (n - self._last[2]) / dt
        if self.ready:
            self.ve += V_EMA_ALPHA * (ve - self.ve)
            self.vn += V_EMA_ALPHA * (vn - self.vn)
        else:
            self.ve, self.vn = ve, vn
            self.ready = True
        self._last = (t, e, n)

    def speed(self) -> float:
        return math.hypot(self.ve, self.vn)


def intercept_t_go(r_e, r_n, v_e, v_n, s):
    """Time-to-go of the constant-velocity lead intercept: smallest positive
    root of (v.v - s^2) t^2 + 2 (r.v) t + r.r = 0, where r = target - drone
    and s is the drone speed cap. None when no positive root exists (target
    as fast as the cap and never closing)."""
    a = v_e * v_e + v_n * v_n - s * s
    b = 2.0 * (r_e * v_e + r_n * v_n)
    c = r_e * r_e + r_n * r_n
    if c == 0.0:
        return 0.0
    if abs(a) < 1e-9:
        return -c / b if b < -1e-9 else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    lo, hi = sorted(((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)))
    if lo > 0.0:
        return lo
    if hi > 0.0:
        return hi
    return None


def control_ref(mode, me_e, me_n, tgt_e, tgt_n, est, speed,
                standoff_e=0.0, standoff_n=0.0):
    """One guidance tick -> (ref_e, ref_n, ff_ve, ff_vn).

    shadow:    ref = target + standoff, feedforward = target velocity
               (PX4's outer P closes the residual -> PD on a moving reference).
    intercept: ref = closed-form lead point, feedforward = speed toward it
               (fire-control geometry recomputed every tick; falls back to a
               full-speed tail-chase while the velocity estimate warms up or
               when no root exists)."""
    if mode == "shadow":
        return (tgt_e + standoff_e, tgt_n + standoff_n, est.ve, est.vn)
    t_go = (intercept_t_go(tgt_e - me_e, tgt_n - me_n, est.ve, est.vn, speed)
            if est.ready else None)
    if t_go is None:
        ref_e, ref_n = tgt_e, tgt_n
    else:
        ref_e = tgt_e + est.ve * t_go
        ref_n = tgt_n + est.vn * t_go
    d = math.hypot(ref_e - me_e, ref_n - me_n)
    if d < 1e-6:
        return ref_e, ref_n, 0.0, 0.0
    return ref_e, ref_n, speed * (ref_e - me_e) / d, speed * (ref_n - me_n) / d


def clamp_ref_alt(world, ref_e, ref_n, alt):
    """Building clamp: a reference inside a footprint below roof+3m is raised
    to roof+3m — pursuit must not wedge the drone into a wall (same rule as
    goto's refusal, but a controller mid-chase clamps instead of erroring)."""
    for b in getattr(world, "buildings", None) or []:
        if (abs(ref_e - b["x"]) <= b["w"] / 2 and abs(ref_n - b["y"]) <= b["d"] / 2
                and alt < b["h"] + 3.0):
            return b["h"] + 3.0
    return alt


class TrackLog:
    """Gap bookkeeping for both modes: min/mean gap and the best CONTIGUOUS
    dwell within `within_m` — horizontal, the same metric as the oracle's
    dwell_moving, so the tool's summary is directly checkable by the LLM."""

    def __init__(self, within_m: float) -> None:
        self.within = within_m
        self.n = 0
        self.min_gap = math.inf
        self._sum = 0.0
        self.best_dwell = 0.0
        self._run_start = None

    def sample(self, t: float, gap: float) -> None:
        self.n += 1
        self._sum += gap
        self.min_gap = min(self.min_gap, gap)
        if gap <= self.within:
            if self._run_start is None:
                self._run_start = t
            self.best_dwell = max(self.best_dwell, t - self._run_start)
        else:
            self._run_start = None

    def mean_gap(self) -> float:
        return self._sum / self.n if self.n else math.inf
```

- [ ] **Step 2: Write `tests/test_track.py` exactly:**

```python
import math

import pytest

from agents.flight.track import (TargetEstimator, TrackLog, clamp_ref_alt,
                                 control_ref, intercept_t_go)


def _fed(samples):
    est = TargetEstimator()
    for t, e, n in samples:
        est.update(t, e, n)
    return est


def test_estimator_constant_velocity():
    est = _fed([(0.0, 0.0, 0.0), (0.1, 0.3, -0.2), (0.2, 0.6, -0.4),
                (0.3, 0.9, -0.6)])
    assert est.ready
    assert est.ve == pytest.approx(3.0, abs=0.01)
    assert est.vn == pytest.approx(-2.0, abs=0.01)
    assert est.speed() == pytest.approx(math.hypot(3.0, 2.0), abs=0.02)


def test_estimator_skips_repeated_stamp():
    est = _fed([(0.0, 0.0, 0.0), (0.0, 5.0, 5.0)])   # same stamp: no velocity
    assert not est.ready
    est.update(0.5, 1.0, 0.0)
    assert est.ready
    assert est.ve == pytest.approx(2.0, abs=0.01)


def test_estimator_smooths_velocity_change():
    # v jumps 2 -> 4 m/s east; EMA moves toward 4 without reaching it in one step
    est = _fed([(0.0, 0.0, 0.0), (1.0, 2.0, 0.0), (2.0, 6.0, 0.0)])
    assert 2.0 < est.ve < 4.0


def test_intercept_t_go_stationary_target():
    # 100m away, target still, speed 10 -> 10s
    assert intercept_t_go(100.0, 0.0, 0.0, 0.0, 10.0) == pytest.approx(10.0)


def test_intercept_t_go_satisfies_collision_equation():
    r_e, r_n, v_e, v_n, s = 100.0, 40.0, 0.0, 4.0, 10.0
    t = intercept_t_go(r_e, r_n, v_e, v_n, s)
    assert t is not None and t > 0
    # at t, target displacement from drone equals s*t
    d = math.hypot(r_e + v_e * t, r_n + v_n * t)
    assert d == pytest.approx(s * t, rel=1e-6)


def test_intercept_t_go_unreachable():
    # target receding at the speed cap: never closes
    assert intercept_t_go(100.0, 0.0, 10.0, 0.0, 10.0) is None


def test_control_ref_shadow_is_target_plus_standoff_with_ff():
    est = _fed([(0.0, 50.0, 0.0), (1.0, 53.0, 0.0)])
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "shadow", 0.0, 0.0, 53.0, 0.0, est, 12.0, standoff_e=-5.0)
    assert (ref_e, ref_n) == (48.0, 0.0)
    assert ff_ve == pytest.approx(3.0, abs=0.01)
    assert ff_vn == pytest.approx(0.0, abs=0.01)


def test_control_ref_intercept_leads_the_target():
    est = _fed([(0.0, 100.0, 0.0), (1.0, 100.0, 4.0)])   # northbound 4 m/s
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "intercept", 0.0, 0.0, 100.0, 4.0, est, 10.0)
    assert ref_n > 4.0                                   # aims AHEAD of the target
    assert math.hypot(ff_ve, ff_vn) == pytest.approx(10.0, rel=1e-6)


def test_control_ref_intercept_fallback_before_estimate():
    est = TargetEstimator()                              # not ready
    ref_e, ref_n, ff_ve, ff_vn = control_ref(
        "intercept", 0.0, 0.0, 60.0, 80.0, est, 12.0)
    assert (ref_e, ref_n) == (60.0, 80.0)                # tail-chase fallback
    assert math.hypot(ff_ve, ff_vn) == pytest.approx(12.0, rel=1e-6)


class _W:
    buildings = [{"name": "b", "x": 0.0, "y": 0.0, "w": 20.0, "d": 20.0, "h": 30.0}]


def test_clamp_ref_alt_raises_inside_footprint():
    assert clamp_ref_alt(_W(), 5.0, -5.0, 12.0) == 33.0


def test_clamp_ref_alt_leaves_clear_refs():
    assert clamp_ref_alt(_W(), 50.0, 0.0, 12.0) == 12.0
    assert clamp_ref_alt(_W(), 0.0, 0.0, 40.0) == 40.0


def test_tracklog_contiguous_dwell_resets():
    log = TrackLog(15.0)
    for t, gap in [(0, 5), (1, 5), (2, 5), (3, 20), (4, 5), (5, 5)]:
        log.sample(float(t), float(gap))
    assert log.best_dwell == pytest.approx(2.0)          # 0..2, reset at t=3
    assert log.min_gap == 5.0
    assert log.mean_gap() == pytest.approx((5 * 5 + 20) / 6)
```

- [ ] **Step 3: Run `uv run --no-project --with pytest pytest tests/test_track.py -q` from the repo root — expect all pass.** (If `uv` is unavailable use `python -m pytest`; the repo's other tests run the same way.)

- [ ] **Step 4: Commit**

```bash
git add agents/flight/track.py tests/test_track.py
git commit -m "feat(flight): guidance core for the track primitive (estimator, lead intercept, dwell log)"
```

### Task 2: FlightOps.track + fleet track_all + MCP tool wiring

**Files:**
- Modify: `agents/flight/ops.py` (add one method + one import block; nothing else changes)
- Modify: `agents/flight/fleet.py` (add `track_all` method)
- Modify: `agents/flight/tools.py` (register `track` in `_drone_server`, `track_all` in `make_operator_options`, extend both system prompts)
- Test: `tests/test_track_tool.py` (create)

**Interfaces:**
- Consumes: everything Task 1 produces (import as `from agents.flight import track as trk`).
- Produces: `async FlightOps.track(target="", mode="shadow", alt=12.0, duration_s=60.0, within_m=15.0, speed=12.0, standoff_east=0.0, standoff_north=0.0) -> str`; `async FleetOps.track_all(tracks: list[dict]) -> str`; MCP tools `mcp__d{i}__track`, `mcp__fleet__track_all`.

- [ ] **Step 1: Add to `agents/flight/ops.py`** — a new method on FlightOps, placed after `set_speed`:

```python
    async def track(self, target="", mode="shadow", alt=12.0, duration_s=60.0,
                    within_m=15.0, speed=12.0, standoff_east=0.0,
                    standoff_north=0.0) -> str:
        """Real-time pursuit of a gz mover: 10 Hz offboard streaming of
        position + velocity-feedforward setpoints (PX4's cascade is the PD
        law — see agents/flight/track.py). Blocks until duration_s (capped)
        elapses, or returns EARLY in intercept mode the moment the horizontal
        gap closes within within_m."""
        import time as _time

        from mavsdk.offboard import (OffboardError, PositionNedYaw,
                                     VelocityNedYaw)

        from agents.flight import track as trk

        if self.gzposes is None:
            raise ValueError("track needs a dynamic world (no mover feed)")
        name = str(target or "").strip()
        poses = self.gzposes.poses()
        if name not in poses:
            known = ", ".join(sorted(poses)) or "none seen yet"
            raise ValueError(f"unknown moving contact {name!r} (visible: {known})")
        mode = str(mode or "shadow").strip().lower()
        if mode not in ("shadow", "intercept"):
            raise ValueError("mode must be 'shadow' or 'intercept'")
        alt = float(alt)
        within = max(1.0, float(within_m))
        speed = min(abs(float(speed)), trk.MAX_SPEED_MPS) or trk.MAX_SPEED_MPS
        dur = min(max(float(duration_s), 1.0), trk.MAX_DURATION_S)
        so_e, so_n = float(standoff_east), float(standoff_north)

        # world ENU -> PX4 local NED: constant offset from one simultaneous read
        me = self.world.world_xy(self.bridge, self.i)
        lp = self.bridge.latest(f"/px4_{self.i}/fmu/out/vehicle_local_position")
        if me is None or lp is None:
            raise ValueError("no position fix yet — take off first")
        off_n, off_e, off_d = lp.x - me[1], lp.y - me[0], lp.z + me[2]

        def _sp(ref_e, ref_n, ref_u, ff_ve, ff_vn, yaw):
            return (PositionNedYaw(ref_n + off_n, ref_e + off_e,
                                   off_d - ref_u, yaw),
                    VelocityNedYaw(ff_vn, ff_ve, 0.0, yaw))

        tp = poses[name]
        yaw = math.degrees(math.atan2(tp[0] - me[0], tp[1] - me[1]))
        pos, vel = _sp(me[0], me[1], alt, 0.0, 0.0, yaw)
        for _ in range(5):                       # prime the stream before start()
            await self.drone.offboard.set_position_velocity_ned(pos, vel)
            await asyncio.sleep(0.05)
        try:
            await self.drone.offboard.start()
        except OffboardError as e:
            raise ValueError(f"offboard start refused: {e._result.result} — "
                             "are you airborne?") from e

        est = trk.TargetEstimator()
        log = trk.TrackLog(within)
        wall0 = _time.monotonic()
        hit = None
        try:
            while _time.monotonic() - wall0 < dur:
                tp = self.gzposes.poses().get(name)
                me = self.world.world_xy(self.bridge, self.i)
                if tp is None or me is None:
                    await asyncio.sleep(1.0 / trk.CTRL_HZ)
                    continue
                est.update(self.gzposes.sim_time(), tp[0], tp[1])
                gap = math.hypot(tp[0] - me[0], tp[1] - me[1])
                log.sample(_time.monotonic() - wall0, gap)
                if mode == "intercept" and gap <= within:
                    hit = (_time.monotonic() - wall0, gap)
                    break
                ref_e, ref_n, ff_ve, ff_vn = trk.control_ref(
                    mode, me[0], me[1], tp[0], tp[1], est, speed, so_e, so_n)
                ref_u = trk.clamp_ref_alt(self.world, ref_e, ref_n, alt)
                yaw = math.degrees(math.atan2(tp[0] - me[0], tp[1] - me[1]))
                pos, vel = _sp(ref_e, ref_n, ref_u, ff_ve, ff_vn, yaw)
                await self.drone.offboard.set_position_velocity_ned(pos, vel)
                await asyncio.sleep(1.0 / trk.CTRL_HZ)
        finally:
            # leave offboard cleanly even on cancellation; Hold keeps position
            try:
                await asyncio.shield(self.drone.offboard.stop())
            except Exception:
                try:
                    await self.drone.action.hold()
                except Exception:
                    pass

        t_total = _time.monotonic() - wall0
        v = (f"target v≈{est.speed():.1f} m/s ({est.ve:+.1f}E {est.vn:+.1f}N)"
             if est.ready else "target velocity not established")
        if mode == "intercept":
            if hit:
                return (f"{self.name} INTERCEPTED {name} at t+{hit[0]:.0f}s, "
                        f"gap {hit[1]:.1f}m; {v}")
            return (f"{self.name} did NOT close within {within:g}m of {name} "
                    f"in {t_total:.0f}s (min gap {log.min_gap:.1f}m); {v}")
        return (f"{self.name} shadowed {name} for {t_total:.0f}s: gap min "
                f"{log.min_gap:.1f}m / mean {log.mean_gap():.1f}m, best "
                f"contiguous ≤{within:g}m: {log.best_dwell:.0f}s; {v}")
```

- [ ] **Step 2: Add to `agents/flight/fleet.py`** after `goto_all` (same error-isolation contract):

```python
    async def track_all(self, tracks: list[dict]) -> str:
        """Concurrent per-drone `track` calls — the pursuit counterpart of
        goto_all (sequential blocking tracks would serialize the fleet and
        blow every timing window)."""
        keys = ("target", "mode", "alt", "duration_s", "within_m", "speed",
                "standoff_east", "standoff_north")
        tasks = []
        for spec in tracks:
            ops = self.drone(spec.get("drone", 0))   # validate BEFORE launching
            kw = {k: spec[k] for k in keys if k in spec}
            tasks.append((spec.get("drone", 0), ops.track(**kw)))
        if not tasks:
            raise ValueError("tracks is empty")
        results = await asyncio.gather(*(t for _, t in tasks),
                                       return_exceptions=True)
        lines = []
        for (i, _), r in zip(tasks, results):
            if isinstance(r, BaseException):
                lines.append(f"drone_{i} track ERROR: {r}")
            else:
                lines.append(str(r))
        return "\n".join(lines)
```

- [ ] **Step 3: Register the `track` tool in `_drone_server` (tools.py)** — add after the `run_mission` tool, include it in the server tool list and `allowed`:

```python
    @tool("track",
          "REAL-TIME PURSUIT of a moving contact (a mov_* from scan): an onboard "
          "10 Hz controller flies the chase for you — far better than chasing "
          "with repeated gotos. mode='shadow' holds station on the moving target "
          "(optional standoff_east/standoff_north offset, metres); "
          "mode='intercept' flies a lead-collision course and returns EARLY the "
          "moment the horizontal gap closes within within_m. Blocks up to "
          "duration_s (max 120s) and reports min/mean gap, best contiguous dwell "
          "within within_m, and the target's measured velocity. You must be "
          "airborne first (take_off).",
          {"target": {"type": "string"}, "mode": {"type": "string"},
           "alt": {"type": "number"}, "duration_s": {"type": "number"},
           "within_m": {"type": "number"}, "speed": {"type": "number"},
           "standoff_east": {"type": "number"}, "standoff_north": {"type": "number"}})
    async def track(args):
        try:
            return _ok(await ops.track(
                args.get("target", ""), args.get("mode", "shadow"),
                args.get("alt", 12.0), args.get("duration_s", 60.0),
                args.get("within_m", 15.0), args.get("speed", 12.0),
                args.get("standoff_east", 0.0), args.get("standoff_north", 0.0)))
        except Exception as e:
            return _err(f"{name} track failed: {e}")
```

Update the `create_sdk_mcp_server(...)` tool list and the `allowed` list to include `track` / `f"mcp__d{i}__track"`.

- [ ] **Step 4: Register `track_all` in `make_operator_options` (tools.py)** — add next to `goto_all` in the fleet server:

```python
    @tool("track_all",
          "Run REAL-TIME PURSUIT on SEVERAL drones at once: tracks=[{drone, "
          "target, mode, alt, duration_s, within_m, speed}, ...] — same "
          "semantics as each drone's `track`, but concurrent (sequential track "
          "calls would leave the other drones parked and blow timing windows). "
          "Returns one summary line per drone.",
          {"tracks": {"type": "array", "items": {"type": "object", "properties": {
              "drone": {"type": "number"}, "target": {"type": "string"},
              "mode": {"type": "string"}, "alt": {"type": "number"},
              "duration_s": {"type": "number"}, "within_m": {"type": "number"},
              "speed": {"type": "number"}}}}})
    async def track_all(args):
        try:
            return _ok(await fleet.track_all(args.get("tracks", [])))
        except Exception as e:
            return _err(f"track_all failed: {e}")
```

`servers["fleet"] = create_sdk_mcp_server(name="fleet", tools=[goto_all, track_all])` and append `"mcp__fleet__track_all"` to `allowed`.

- [ ] **Step 5: Prompt lines.** In `make_drone_options`' system prompt, after the MOVE paragraph, add:

```
"TRACK: for a MOVING contact (mov_* in scan), `track(target, mode, alt, "
"duration_s, within_m)` runs an onboard real-time pursuit controller — "
"mode='shadow' to stay on it (dwell tasks), mode='intercept' to close on it "
"fast (returns early on contact). One call beats any sequence of gotos at "
"following a mover; verify its returned gap/dwell numbers against your "
"task before reporting success.\n"
```

In `make_operator_options`' system prompt, after the goto_all sentence in the first paragraph, add:

```
"mcp__fleet__track_all runs real-time pursuit (shadow/intercept) on several "
"drones AT ONCE — for simultaneous moving-target work, one track_all call "
"beats interleaving anything by hand.\n"
```

- [ ] **Step 6: Write `tests/test_track_tool.py`** — follow the existing style of `tests/test_operator_tools.py` / `tests/test_fleet_ops.py` (read them first) to cover, with fakes and NO sim: (a) `FleetOps.track_all` fans out concurrently and isolates one drone's exception as a `track ERROR:` line while the other's summary survives (fake ops objects whose `track` returns/raises); (b) `_drone_server` now exposes `mcp__d0__track` in its allowed list and `make_operator_options`' allowed list contains `mcp__fleet__track_all` (mirror how existing tests assert tool registration — if they construct options with fake systems, reuse that fixture pattern); (c) `FlightOps.track` raises ValueError on unknown mover and on `gzposes=None` (construct FlightOps with `drone=None, world=None, bridge=None` and a fake gzposes exposing `poses()`/`sim_time()` — the validation happens before any drone/world access).

- [ ] **Step 7: Run the new + neighbouring tests:** `uv run --no-project --with pytest pytest tests/test_track.py tests/test_track_tool.py tests/test_fleet_ops.py tests/test_drone_tools.py tests/test_operator_tools.py -q` — all pass.

- [ ] **Step 8: Commit**

```bash
git add agents/flight/ops.py agents/flight/fleet.py agents/flight/tools.py tests/test_track_tool.py
git commit -m "feat(flight): track/track_all — LLM-parameterized classical pursuit via PX4 offboard"
```

### Task 3: Live smoke on evals-dyn (controller vs real PX4)

Run by the session controller (needs live-tuning judgment). Inside `evals-dyn` (SWARM_N=1, GZ_WORLD=dynamic): a python script that builds RosBridge/World/GzPoses + a MAVSDK System exactly the way `evals/run_evals.py` does, takes off, then (a) `track(mov_1, shadow, 60s, within 15)` — expect mean gap < 8 m and best dwell ≥ 45 s; (b) `track(mov_3, intercept, within 10, speed 12)` — expect INTERCEPTED. Bounded: whole script under `timeout 300`. If gaps oscillate (stop-start chatter) the first knob is feedforward correctness, NOT gains — verify the ENU→NED mapping and that velocity feedforward is nonzero. Acceptance: both summaries meet the numbers above; record them in the ledger.

### Task 4: Track-based pilots + dual-baseline gates (d2, d4, w4)

**Files:**
- Modify: `evals/tasks/dynamic/d2_shadow.yaml` (pilot block ONLY)
- Modify: `evals/tasks/dynamic/d4_estimate_intercept.yaml` (pilot block ONLY)
- Modify: `evals/tasks/swarm/w4_double_intercept.yaml` (pilot block ONLY)

New pilots (null_pilots, oracles, prompts, budgets FROZEN):

d2: `take_off alt 12` → `{tool: track, args: {target: mov_1, mode: shadow, alt: 12, duration_s: 75, within_m: 15}}` (75 s > 45 s dwell + convergence).
d4: `take_off alt 12` → `{tool: track, args: {target: mov_3, mode: intercept, alt: 12, within_m: 10, speed: 12, duration_s: 90}}`.
w4: two take_offs → `{tool: track_all, args: {tracks: [{drone: 0, target: mov_0, mode: intercept, alt: 12, within_m: 12, speed: 12, duration_s: 110}, {drone: 1, target: mov_1, mode: intercept, alt: 14, within_m: 12, speed: 12, duration_s: 110}]}}` (different altitudes: separation hygiene).

Keep a one-line YAML comment above each pilot noting the previous pilot strategy it replaced (trajectory-authoring / ambush) and why (track primitive is now the ideal toolpath). Note the pilot runner resolves `{tool: ...}` steps against fleet-then-drone (`evals/pilot.py`): `track` hits the drone ops, `track_all` the fleet — verify `track_all` is reachable for the w4 pilot (FleetOps has it after Task 2; the pilot targets ops objects directly, not MCP).

Gates (bounded, `timeout 3600` each): d2+d4 on evals-dyn (`SWARM_N=1 GZ_WORLD=dynamic`), w4 on evals-fleetdyn (`SWARM_N=2 GZ_WORLD=dynamic`), command shape: `python -m evals.run_evals --tasks <task.yaml> --pilot --k 2 --seed 11 --out evals/out/pilot_track/<task>`. Reading: pilot rows PASS 2/2, null rows FAIL 2/2, per task. Fix policy: max 2 iterations, pilot-args-only calibration (duration, alt, within_m); NEVER touch oracle/null/budget; else BLOCKED with diagnosis.

Commit YAMLs + gate dirs: `git add evals/tasks evals/out/pilot_track && git commit -m "feat(evals): track-based pilots + gates for d2/d4/w4"`.

### Task 5: Tier sweep with track + report

1. evals-dyn: `--tasks d2_shadow.yaml d4_estimate_intercept.yaml --assignments "drones=opus;drones=sonnet;drones=haiku" --k 2 --seed 11 --out evals/out/track_dyn` (12 cells, timeout 10800).
2. evals-fleetdyn: same assignments on `w4_double_intercept.yaml`, `--out evals/out/track_w4` (6 cells, timeout 7200).
3. Compare with the pre-track control arm (committed data: first dynamic sweep + E1 w4 rows). Write `docs/benchmarks/EVALS-TRACK-2026-07-06.md`: per-task per-tier pass deltas, the two headline questions from the design doc, controller behavior notes (gaps from transcripts), cost notes.
4. Commit data + report; update ledger + memory (`dynamic-scenarios-suite.md` pointer to track results).

---

## Self-review notes
- Task 1/2 code is complete and mutually consistent (`trk.` prefixes, signatures match the tool wrappers; `math`/`asyncio` already imported at ops.py top level; `time` imported locally as `_time` to avoid touching module imports).
- Pilot runner compatibility: behaviors/tool steps route via `ops` attr lookup (`target = ops if hasattr(ops, tool) else ops.drone(...)`) — FleetOps gains `track_all` (fleet-level) and FlightOps gains `track` (drone-level), so both resolve with zero pilot.py changes.
- The d2 null (lead_chaser) and d4/w4 nulls are untouched and still must fail — the tasks keep discriminating; what changes is that the IDEAL toolpath no longer requires trajectory authoring.
