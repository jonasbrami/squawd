# run_mission Code Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add one MCP tool `run_mission(code, timeout)` that lets a drone-Claude author a full async MAVSDK mission in Python, exec'd in-process against the live connection, with the result/traceback fed back so it can iterate.

**Architecture:** A module-level `_mission_item(**kw)` factory (nan/NONE-defaulting `MissionItem` passthrough) plus a `FlightOps.run_mission` async harness that wraps Claude's code as an `async def` body, exec's it with an injected namespace (`drone`, `mission_item`, `world_to_geo`, `log`), awaits it under a Claude-set timeout, halts the vehicle on timeout, and returns `(is_error, text)`. A thin `run_mission` tool wrapper in `tools.py` registers it alongside the existing 8 primitives (which are untouched).

**Tech Stack:** Python 3.10+, mavsdk (MissionItem/MissionPlan/mission plugin), claude-agent-sdk (`@tool`, `create_sdk_mcp_server`, `ClaudeAgentOptions`), pytest + pytest-asyncio (`asyncio_mode = "auto"`), uv.

## Global Constraints

- Python `>=3.10`; mavsdk `>=2.0.0`; claude-agent-sdk `>=0.2.0` (verbatim from `pyproject.toml`).
- Run tests from the worktree with: `uv run --extra dev pytest <path> -q` (builds against the worktree, not the main checkout).
- Do NOT modify the 8 existing flight primitives (`take_off`, `fly`, `goto`, `orbit`, `hover`, `set_speed`, `face`, `land`) or `look`/`scan`/`report`. `run_mission` is added alongside them.
- Injected helpers must be non-limiting: real SDK field names, every field overridable, no clamps, no hidden policy. No altitude/velocity caps in the harness.
- `MissionItem` constructor (exact 14 fields, all required, verified against the install): `latitude_deg, longitude_deg, relative_altitude_m, speed_m_s, is_fly_through, gimbal_pitch_deg, gimbal_yaw_deg, camera_action, loiter_time_s, camera_photo_interval_s, acceptance_radius_m, yaw_deg, camera_photo_distance_m, vehicle_action`. Enums: `MissionItem.CameraAction.NONE`, `MissionItem.VehicleAction.NONE`. `MissionPlan(mission_items)`.
- Coordinate rule: `world_to_geo` returns AMSL `absolute_altitude_m`; `MissionItem.relative_altitude_m` is relative to home. Use `world_to_geo` for lat/lon ONLY; set `relative_altitude_m` to the world `up`.

---

### Task 1: `_mission_item` factory

**Files:**
- Modify: `agents/flight/ops.py` (add `MissionItem` import; add module-level `_mission_item`)
- Test: `tests/test_flight_helpers.py` (create)

**Interfaces:**
- Consumes: `mavsdk.mission.MissionItem`.
- Produces: `agents.flight.ops._mission_item(**kw) -> MissionItem` — every field defaults to `float('nan')`, `is_fly_through` defaults `True`, `camera_action`/`vehicle_action` default to their `.NONE` enums; any keyword (using the SDK's own field names) overrides its default.

- [ ] **Step 1: Write the failing test**

Create `tests/test_flight_helpers.py`:

```python
import math

from mavsdk.mission import MissionItem

from agents.flight.ops import _mission_item


def test_mission_item_defaults_are_nan_and_enums_none():
    it = _mission_item()
    assert math.isnan(it.latitude_deg)
    assert math.isnan(it.longitude_deg)
    assert math.isnan(it.relative_altitude_m)
    assert math.isnan(it.speed_m_s)
    assert it.is_fly_through is True
    assert it.camera_action == MissionItem.CameraAction.NONE
    assert it.vehicle_action == MissionItem.VehicleAction.NONE


def test_mission_item_overrides_apply():
    it = _mission_item(latitude_deg=1.5, longitude_deg=2.5,
                       relative_altitude_m=15.0, speed_m_s=5.0,
                       is_fly_through=False)
    assert it.latitude_deg == 1.5
    assert it.longitude_deg == 2.5
    assert it.relative_altitude_m == 15.0
    assert it.speed_m_s == 5.0
    assert it.is_fly_through is False
    # untouched fields keep their nan default
    assert math.isnan(it.yaw_deg)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run --extra dev pytest tests/test_flight_helpers.py -q`
Expected: FAIL — `ImportError: cannot import name '_mission_item' from 'agents.flight.ops'`

- [ ] **Step 3: Add the import**

In `agents/flight/ops.py`, directly below the existing line `from mavsdk.action import OrbitYawBehavior` (line 16), add:

```python
from mavsdk.mission import MissionItem
```

- [ ] **Step 4: Add the factory**

In `agents/flight/ops.py`, directly after the `COMPASS = {...}` dict (ends line 23) and before `class FlightOps:`, add:

```python
def _mission_item(**kw):
    """A MissionItem with every field defaulted (nan / enum NONE), overridable by
    its real SDK field name. Cuts the 14-required-arg boilerplate; hides nothing."""
    nan = float("nan")
    fields = dict(
        latitude_deg=nan, longitude_deg=nan, relative_altitude_m=nan,
        speed_m_s=nan, is_fly_through=True,
        gimbal_pitch_deg=nan, gimbal_yaw_deg=nan,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=nan, camera_photo_interval_s=nan,
        acceptance_radius_m=nan, yaw_deg=nan, camera_photo_distance_m=nan,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )
    fields.update(kw)
    return MissionItem(**fields)
```

- [ ] **Step 5: Run test to verify it passes**

Run: `uv run --extra dev pytest tests/test_flight_helpers.py -q`
Expected: PASS (2 passed)

- [ ] **Step 6: Commit**

```bash
git add agents/flight/ops.py tests/test_flight_helpers.py
git commit -m "feat(flight): _mission_item nan/NONE-defaulting MissionItem factory"
```

---

### Task 2: `run_mission` exec harness

**Files:**
- Modify: `agents/flight/ops.py` (add `textwrap`/`traceback` imports, `DEFAULT_MISSION_TIMEOUT_S`, `_result_text` helper, `FlightOps._halt`, `FlightOps.run_mission`)
- Test: `tests/test_run_mission.py` (create)

**Interfaces:**
- Consumes: `_mission_item` (Task 1); `self.drone` (a MAVSDK `System`); `self._world_to_geo`; `self.name`.
- Produces:
  - `agents.flight.ops.DEFAULT_MISSION_TIMEOUT_S: float` (= `180.0`).
  - `FlightOps.run_mission(code: str, timeout=None) -> tuple[bool, str]` — `(is_error, text)`. Execs `code` as an async body with namespace `{drone, mission_item, world_to_geo, log}`; on success returns `(False, logs+return-value-text)`; on snippet/compile exception returns `(True, logs+traceback)`; on timeout halts the vehicle and returns `(True, logs+"timed out … vehicle halted")`. `timeout` is seconds; `None` → `DEFAULT_MISSION_TIMEOUT_S`.
  - `FlightOps._halt() -> None` — `await drone.mission.pause_mission()`, falling back to `drone.action.hold()`, swallowing errors.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_run_mission.py`:

```python
import math

from agents.flight.ops import FlightOps, DEFAULT_MISSION_TIMEOUT_S


class FakeMission:
    def __init__(self):
        self.uploaded = None
        self.started = False
        self.paused = False

    async def upload_mission(self, plan):
        self.uploaded = plan

    async def start_mission(self):
        self.started = True

    async def pause_mission(self):
        self.paused = True


class FakeAction:
    def __init__(self):
        self.armed = False
        self.held = False

    async def arm(self):
        self.armed = True

    async def hold(self):
        self.held = True


class FakeDrone:
    def __init__(self):
        self.mission = FakeMission()
        self.action = FakeAction()


def _ops():
    return FlightOps(FakeDrone(), world=None, bridge=None, i=0, n=1)


async def test_success_returns_logs_and_value():
    err, text = await _ops().run_mission('log("hello")\nreturn "done"')
    assert err is False
    assert "hello" in text
    assert "done" in text


async def test_runs_full_lifecycle_against_drone():
    ops = _ops()
    code = (
        "from mavsdk.mission import MissionPlan\n"
        "items = [mission_item(latitude_deg=1.0, longitude_deg=2.0, "
        "relative_altitude_m=15.0, speed_m_s=5.0)]\n"
        "await drone.mission.upload_mission(MissionPlan(items))\n"
        "await drone.action.arm()\n"
        "await drone.mission.start_mission()\n"
        "return 'flown'"
    )
    err, text = await ops.run_mission(code)
    assert err is False
    assert ops.drone.action.armed is True
    assert ops.drone.mission.started is True
    assert ops.drone.mission.uploaded is not None
    assert "flown" in text


async def test_none_return_reports_completed():
    err, text = await _ops().run_mission("x = 1 + 1")
    assert err is False
    assert "completed" in text.lower()


async def test_runtime_exception_returns_traceback():
    err, text = await _ops().run_mission("raise ValueError('boom')")
    assert err is True
    assert "ValueError" in text
    assert "boom" in text


async def test_syntax_error_returns_error_not_raise():
    err, text = await _ops().run_mission("def : this is not python")
    assert err is True
    assert "Error" in text


async def test_timeout_halts_vehicle():
    ops = _ops()
    err, text = await ops.run_mission("import asyncio\nawait asyncio.sleep(100)",
                                      timeout=0.05)
    assert err is True
    assert "timed out" in text
    assert ops.drone.mission.paused is True


def test_default_timeout_value():
    assert DEFAULT_MISSION_TIMEOUT_S == 180.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run --extra dev pytest tests/test_run_mission.py -q`
Expected: FAIL — `ImportError: cannot import name 'run_mission' / 'DEFAULT_MISSION_TIMEOUT_S'` (and `FlightOps` has no `run_mission`).

- [ ] **Step 3: Add module imports and constants**

In `agents/flight/ops.py`, the existing top imports are `import asyncio` / `import math` (lines 13-14). Add below them:

```python
import textwrap
import traceback
```

Then, directly after the `_mission_item` factory added in Task 1, add:

```python
DEFAULT_MISSION_TIMEOUT_S = 180.0


def _result_text(logs, body):
    """Prefix any log() lines before the result/traceback body."""
    if logs:
        return "logs:\n" + "\n".join(logs) + "\n\n" + body
    return body
```

- [ ] **Step 4: Add `_halt` and `run_mission` to FlightOps**

In `agents/flight/ops.py`, add these methods to the `FlightOps` class, after `scan` (the current last method, ends line 163):

```python
    async def _halt(self) -> None:
        """Stop the vehicle after a cancelled/timed-out mission: cancelling the
        Python coroutine does NOT stop PX4 flying the already-uploaded mission."""
        try:
            await self.drone.mission.pause_mission()
        except Exception:
            try:
                await self.drone.action.hold()
            except Exception:
                pass

    async def run_mission(self, code: str, timeout=None):
        """Exec a Claude-authored async MAVSDK body in-process; return (is_error, text).

        Namespace: `drone` (live System), `mission_item(**fields)`, `world_to_geo`
        (await -> GeoPoint), `log(msg)`. Claude imports MAVSDK classes itself.
        `timeout` (s) is Claude-set; None -> DEFAULT_MISSION_TIMEOUT_S. On timeout
        the vehicle is halted before the error is returned."""
        logs = []
        ns = {
            "drone": self.drone,
            "mission_item": _mission_item,
            "world_to_geo": self._world_to_geo,
            "log": logs.append,
        }
        src = "async def _snippet():\n" + textwrap.indent(code or "", "    ")
        t = float(timeout) if timeout is not None else DEFAULT_MISSION_TIMEOUT_S
        try:
            exec(compile(src, "<mission>", "exec"), ns)
            ret = await asyncio.wait_for(ns["_snippet"](), timeout=t)
        except asyncio.TimeoutError:
            await self._halt()
            return True, _result_text(
                logs, f"{self.name}: mission timed out after {t:g}s; vehicle halted")
        except Exception:
            return True, _result_text(logs, traceback.format_exc())
        body = f"{self.name}: completed (no return value)" if ret is None else str(ret)
        return False, _result_text(logs, body)
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/test_run_mission.py -q`
Expected: PASS (7 passed)

- [ ] **Step 6: Commit**

```bash
git add agents/flight/ops.py tests/test_run_mission.py
git commit -m "feat(flight): run_mission in-process exec harness with halt-on-timeout"
```

---

### Task 3: Register the `run_mission` MCP tool

**Files:**
- Modify: `agents/flight/tools.py` (add the `run_mission` tool wrapper; add to `tools=[...]` and `allowed_tools=[...]`; extend the system prompt)
- Test: `tests/test_drone_tools.py` (create)

**Interfaces:**
- Consumes: `FlightOps.run_mission` (Task 2); existing `make_drone_options`, `_ok`, `_err`.
- Produces: a tool named `run_mission` whose MCP id `mcp__d{i}__run_mission` is present in the returned `ClaudeAgentOptions.allowed_tools`; the tool returns `_err(text)` when `run_mission` reports an error, else `_ok(text)`.

- [ ] **Step 1: Write the failing test**

Create `tests/test_drone_tools.py`:

```python
from agents.flight import make_drone_options


def test_run_mission_tool_registered_and_allowed():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "mcp__d0__run_mission" in opts.allowed_tools


def test_existing_primitives_still_registered():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    for name in ("take_off", "goto", "orbit", "land", "look", "scan", "report"):
        assert f"mcp__d0__{name}" in opts.allowed_tools


def test_system_prompt_mentions_run_mission():
    opts = make_drone_options(0, None, None, None, 1, None, lambda m: None)
    assert "run_mission" in opts.system_prompt
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run --extra dev pytest tests/test_drone_tools.py -q`
Expected: FAIL — `test_run_mission_tool_registered_and_allowed` and `test_system_prompt_mentions_run_mission` fail (id/text absent); the primitives test passes.

- [ ] **Step 3: Add the tool wrapper**

In `agents/flight/tools.py`, directly before the `server = create_sdk_mcp_server(` call (line 115), add:

```python
    @tool("run_mission",
          "Author and run your OWN async MAVSDK mission code for a multi-leg or "
          "smooth trajectory. Pre-bound (no import): `drone` (the connected System), "
          "`mission_item(**fields)` (a MissionItem with every field defaulted, "
          "overridable by its real name), `await world_to_geo(east, north, up)` "
          "(-> GeoPoint; use its lat/lon ONLY, set relative_altitude_m to the world "
          "`up`), `log(msg)`. Import MAVSDK classes you use (e.g. MissionPlan). "
          "Set `timeout` to the seconds you expect; you are uninterruptible until "
          "the mission finishes or the timeout fires (which halts you).",
          {"code": {"type": "string"}, "timeout": {"type": "number"}})
    async def run_mission(args):
        try:
            err, text = await ops.run_mission(args.get("code", ""), args.get("timeout"))
            return _err(text) if err else _ok(text)
        except Exception as e:
            return _err(f"{name} run_mission failed: {e}")
```

- [ ] **Step 4: Register the tool**

In `agents/flight/tools.py`, in the `create_sdk_mcp_server(...)` call (line 115-117), add `run_mission` to the `tools=[...]` list (after `scan`):

```python
    server = create_sdk_mcp_server(
        name=f"d{i}", tools=[take_off, fly, goto, orbit, hover, set_speed, face, land,
                             report_tool, look, scan, run_mission])
```

In the `allowed_tools=[...]` list (lines 120-123), add the new id (after `mcp__d{i}__scan`):

```python
                       f"mcp__d{i}__look", f"mcp__d{i}__scan", f"mcp__d{i}__run_mission"],
```

- [ ] **Step 5: Extend the system prompt**

In `agents/flight/tools.py`, the `system_prompt=(...)` block ends with the `SENSE:` paragraph (lines 136-139), whose final string literal is:

```python
            "or `orbit` it, then `look`. Use `scan` before moving near obstacles."),
```

Replace that single line with the same text plus a MISSION paragraph (note the closing `)` moves to the new last line):

```python
            "or `orbit` it, then `look`. Use `scan` before moving near obstacles.\n"
            "MISSION: for a multi-leg or smooth trajectory, `run_mission(code, timeout)` "
            "runs your OWN async MAVSDK. Pre-bound (no import): `drone`, "
            "`mission_item(**fields)`, `await world_to_geo(east,north,up)`, `log(msg)`; "
            "import MAVSDK classes (e.g. MissionPlan) yourself. Coords: lat/lon from "
            "`world_to_geo`, set `relative_altitude_m` to the world `up` (NOT its "
            "absolute altitude). Example:\n"
            "  from mavsdk.mission import MissionPlan\n"
            "  pts = [(0,0,15), (40,0,15), (40,40,15)]\n"
            "  items = []\n"
            "  for e,n_,u in pts:\n"
            "      g = await world_to_geo(east=e, north=n_, up=u)\n"
            "      items.append(mission_item(latitude_deg=g.latitude_deg, "
            "longitude_deg=g.longitude_deg, relative_altitude_m=u, speed_m_s=5, "
            "is_fly_through=True))\n"
            "  await drone.mission.upload_mission(MissionPlan(items))\n"
            "  await drone.action.arm(); await drone.mission.start_mission()\n"
            "  async for p in drone.mission.mission_progress():\n"
            "      log(f'{p.current}/{p.total}')\n"
            "      if p.current == p.total: break\n"
            "  return 'mission complete'\n"
            "Set `timeout` to the seconds the path needs."),
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `uv run --extra dev pytest tests/test_drone_tools.py -q`
Expected: PASS (3 passed)

- [ ] **Step 7: Run the full suite**

Run: `uv run --extra dev pytest tests/ -q`
Expected: PASS (39 passed — 27 baseline + 2 + 7 + 3)

- [ ] **Step 8: Commit**

```bash
git add agents/flight/tools.py tests/test_drone_tools.py
git commit -m "feat(flight): register run_mission MCP tool + mission prompt block"
```

---

### Task 4: Sim integration verification (manual, time-bounded)

Not automated (needs PX4 + Gazebo). Run once to confirm the end-to-end path. **Hard cap the whole check at ~20 minutes**; if the sim isn't up by then, stop and report rather than waiting.

**Files:** none (verification only).

- [ ] **Step 1: Launch the sim swarm** per the project README (the same flow used for the existing primitives), with at least 1 drone and the Commander.

- [ ] **Step 2: Task a drone with a multi-leg path**, e.g. via the Commander: "Fly a 40m square at 15m altitude and report." Confirm the drone calls `run_mission` (visible in its tool stream) rather than chaining `goto`.

- [ ] **Step 3: Observe** the drone flies the waypoints smoothly (fly-through, no stop-and-go) and `report(...)` returns a sane summary; the `mission_progress` log lines appear in the result.

- [ ] **Step 4: Negative check** — task something that makes Claude write a broken snippet (or temporarily feed a bad one), and confirm the traceback comes back as an error result and Claude retries with corrected code.

- [ ] **Step 5: Timeout check** — task a mission with a deliberately tiny `timeout`; confirm the vehicle halts (holds/pauses) and the result says it timed out.

- [ ] **Step 6: Record the outcome** in the PR description (what was tasked, what flew, screenshots/log excerpts). No commit needed.

---

## Self-Review

**Spec coverage:**
- Tool `run_mission(code, timeout)` → Task 3 (wrapper) + Task 2 (harness). ✓
- In-process exec of async body, shared connection → Task 2. ✓
- Injected namespace `drone` + `mission_item` + `world_to_geo` + `log` → Task 2 (harness ns) + Task 1 (`mission_item`). ✓
- Full-lifecycle snippet, Claude writes imports → prompt example (Task 3) + harness allows imports (Task 2). ✓
- Claude-set timeout, default if unset, no ceiling → Task 2 (`timeout`/`DEFAULT_MISSION_TIMEOUT_S`) + Task 3 schema. ✓
- Halt-on-timeout (S2) → Task 2 `_halt` + `test_timeout_halts_vehicle`. ✓
- `log()` capture, no global stdout (S3) → Task 2 ns `log` + `_result_text`. ✓
- Coordinate/altitude rule (B2) → Global Constraints + prompt text (Task 3). ✓
- Result feedback incl. traceback / None-return → Task 2 tests. ✓
- Primitives + look/scan/report untouched → `test_existing_primitives_still_registered` (Task 3); no edits to those methods. ✓
- Accepted risks (loop-block S1, warning-only geofence S4, no clamp) → no code (documented in spec); nothing in plan contradicts them. ✓
- Integration sim test, time-bounded → Task 4. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code; every test step shows the assertion and the run command + expected output.

**Type consistency:** `run_mission` returns `(is_error: bool, text: str)` in Task 2 and is consumed as `err, text = await ops.run_mission(...)` in Task 3. `_mission_item(**kw)` defined in Task 1, used in Task 2 ns and the Task 3 prompt example. `DEFAULT_MISSION_TIMEOUT_S` defined and asserted consistently. Helper names `_result_text` / `_halt` used only where defined.
