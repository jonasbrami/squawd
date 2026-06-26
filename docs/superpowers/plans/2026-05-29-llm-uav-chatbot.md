# LLM-Piloted UAV Chatbot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a terminal chatbot that pilots a simulated UAV in natural language ("take off, fly 50m north, what do you see?, land"), with a modular onboard-perception layer, on PX4 SITL + Gazebo.

**Architecture:** Single asyncio process. A Claude Agent SDK client drives in-process `@tool` adapters that call a plain-Python `CommandExecutor` (the portable, SDK-agnostic command boundary). `CommandExecutor` uses a `DroneController` (MAVSDK) for fire-and-monitor flight actions, an authoritative `StateStore` fed by a background telemetry task, and a non-bypassable `SafetyGuard`. A parallel `PerceptionProvider` (Gazebo sensors now, swappable later) feeds a `PerceptionStore`. A terminal REPL renders from the stores and owns a direct, non-LLM abort path.

**Tech Stack:** Python ≥3.10, `claude-agent-sdk`, `mavsdk` (MAVSDK-Python), PX4 SITL, Gazebo Harmonic, `pytest` + `pytest-asyncio`.

**Reference spec:** `docs/superpowers/specs/2026-05-29-llm-uav-chatbot-design.md`

---

## File Structure

| Path | Responsibility |
|---|---|
| `pyproject.toml` | Project metadata, deps, pytest config (`asyncio_mode = "auto"`). |
| `.env.example` | `ANTHROPIC_API_KEY`, connection URL, model name. |
| `scripts/run_sim.sh` | Launch PX4 SITL + Gazebo with the camera/depth model + safety params. |
| `src/squawd/config.py` | The single sim-vs-real seam: limits, geofence, connection URL, model, sensor selection. Fail-closed defaults. |
| `src/squawd/control/geo.py` | Pure geodetic offset math. NED-frame conventions. No I/O. |
| `src/squawd/control/safety.py` | `SafetyGuard` + `SafetyLimits` + `DroneSnapshot`. Pure invariants. No I/O. |
| `src/squawd/control/state.py` | `StateStore` — authoritative drone state. |
| `src/squawd/control/telemetry.py` | Background task: MAVSDK streams → `StateStore`. |
| `src/squawd/control/controller.py` | `DroneController` — MAVSDK wrapper, fire-and-monitor actions, timeouts. |
| `src/squawd/control/executor.py` | `CommandExecutor` — portable command boundary; combines controller + safety + state. |
| `src/squawd/perception/provider.py` | `PerceptionProvider` interface + `PerceptionSnapshot`. |
| `src/squawd/perception/gazebo_perception.py` | v1 sensor reader (Gazebo). |
| `src/squawd/perception/store.py` | `PerceptionStore` — latest snapshot. |
| `src/squawd/agent/tools.py` | `@tool` adapters → `CommandExecutor` / `PerceptionStore`; return `is_error` on failure. |
| `src/squawd/agent/prompts.py` | System prompt. |
| `src/squawd/agent/claude_agent.py` | `ClaudeSDKClient` wiring + options. |
| `src/squawd/chat/repl.py` | Async stdin, status rendering, direct abort. |
| `src/squawd/flight_log.py` | Structured flight-record (utterance→tool→safety→result→telemetry). |
| `src/squawd/app.py` | Owns the single event loop; wires all layers; entrypoint. |
| `spikes/loop_spike.py` | Throwaway go/no-go: MAVSDK coroutine awaited inside a Claude SDK in-process tool. |

---

## Milestone 0 — Project scaffolding & environment

### Task 1: Python project skeleton

**Files:**
- Create: `pyproject.toml`
- Create: `src/squawd/__init__.py` (empty)
- Create: `src/squawd/control/__init__.py` (empty)
- Create: `src/squawd/perception/__init__.py` (empty)
- Create: `src/squawd/agent/__init__.py` (empty)
- Create: `src/squawd/chat/__init__.py` (empty)
- Create: `tests/__init__.py` (empty)
- Create: `.env.example`
- Create: `.gitignore`

- [ ] **Step 1: Create `pyproject.toml`**

```toml
[project]
name = "squawd"
version = "0.1.0"
description = "LLM-piloted UAV chatbot (simulation)"
requires-python = ">=3.10"
dependencies = [
    "mavsdk>=2.0.0",
    "claude-agent-sdk>=0.2.0",
    "python-dotenv>=1.0.0",
]

[project.optional-dependencies]
dev = ["pytest>=8.0", "pytest-asyncio>=0.23"]

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
asyncio_mode = "auto"
testpaths = ["tests"]
pythonpath = ["src"]
```

- [ ] **Step 2: Create `.gitignore`**

```gitignore
__pycache__/
*.pyc
.venv/
.env
*.log
flight_logs/
.pytest_cache/
```

- [ ] **Step 3: Create `.env.example`**

```bash
ANTHROPIC_API_KEY=sk-ant-...
# MAVSDK connection to PX4 SITL (PX4 offboard API port)
SQUAWD_CONNECTION_URL=udp://:14540
SQUAWD_MODEL=claude-opus-4-8
```

- [ ] **Step 4: Create the empty package files**

Create each `__init__.py` listed above as an empty file.

- [ ] **Step 5: Create and verify the virtualenv**

Run:
```bash
python3 -m venv .venv && . .venv/bin/activate && pip install -e ".[dev]"
```
Expected: installs `mavsdk`, `claude-agent-sdk`, `pytest`, `pytest-asyncio` with no errors.

- [ ] **Step 6: Verify imports + empty test run**

Run:
```bash
. .venv/bin/activate && python -c "import mavsdk, claude_agent_sdk; print('ok')" && pytest -q
```
Expected: prints `ok`; pytest reports `no tests ran`.

- [ ] **Step 7: Commit**

```bash
git add pyproject.toml .gitignore .env.example src tests
git commit -m "chore: scaffold squawd python project"
```

### Task 2: Simulator launch script

**Files:**
- Create: `scripts/run_sim.sh`

> **Prerequisite (manual, one-time):** PX4-Autopilot cloned and built for SITL, and Gazebo **Harmonic** installed (Ubuntu 22.04 or 24.04). Follow the official PX4 toolchain setup (`Tools/setup/ubuntu.sh` in the PX4-Autopilot repo, then `make px4_sitl gz_x500_depth` once to fetch Gazebo assets). Garden (EOL Nov 2024) and Gazebo-Classic are **not** supported here.

- [ ] **Step 1: Create `scripts/run_sim.sh`**

```bash
#!/usr/bin/env bash
# Launch PX4 SITL + Gazebo Harmonic with the depth-camera airframe.
# Usage: PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_sim.sh
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
MODEL="${PX4_SIM_MODEL:-gz_x500_depth}"

if [[ ! -d "$PX4_DIR" ]]; then
  echo "PX4-Autopilot not found at $PX4_DIR. Set PX4_DIR." >&2
  exit 1
fi

echo "Launching PX4 SITL + Gazebo ($MODEL) from $PX4_DIR ..."
cd "$PX4_DIR"
make px4_sitl "$MODEL"
```

- [ ] **Step 2: Make it executable**

Run: `chmod +x scripts/run_sim.sh`

- [ ] **Step 3: Verify the sim launches**

Run: `PX4_DIR=$HOME/PX4-Autopilot ./scripts/run_sim.sh`
Expected: Gazebo window opens showing an x500 quadcopter; the PX4 shell prints `INFO [commander] Ready for takeoff!` within ~30s. Leave it running for later tasks. (If headless, set `HEADLESS=1` per PX4 docs.)

- [ ] **Step 4: Commit**

```bash
git add scripts/run_sim.sh
git commit -m "chore: add PX4 SITL + Gazebo launch script"
```

---

## Milestone 1 — Event-loop spike (GO / NO-GO GATE)

> **This milestone decides whether the whole in-process design holds.** Do not proceed to Milestone 3 until Task 4 passes. Task 3 is a cheap, no-API check that MAVSDK talks to SITL on our loop; Task 4 is the real test: a MAVSDK coroutine awaited from inside a Claude Agent SDK in-process tool on the **same** loop.

### Task 3: MAVSDK ↔ SITL connectivity check (no LLM, no API key)

**Files:**
- Create: `spikes/mavsdk_check.py`

- [ ] **Step 1: Write the connectivity script**

```python
# spikes/mavsdk_check.py
"""Cheap check: connect to PX4 SITL and read one telemetry value.
Requires the sim running (scripts/run_sim.sh). No API key needed.
"""
import asyncio
from mavsdk import System


async def main() -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("connected")
            break

    async for position in drone.telemetry.position():
        print(
            f"lat={position.latitude_deg:.6f} "
            f"lon={position.longitude_deg:.6f} "
            f"rel_alt={position.relative_altitude_m:.2f}m"
        )
        break

    print("SPIKE A PASS: MAVSDK <-> SITL works on this loop")


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 2: Run it against the sim**

Run (with the sim from Task 2 running): `. .venv/bin/activate && python spikes/mavsdk_check.py`
Expected: prints `connected`, then a lat/lon/alt line, then `SPIKE A PASS`. If you see `got Future <...> attached to a different loop`, stop and record it — that is the risk this milestone exists to surface.

- [ ] **Step 3: Commit**

```bash
git add spikes/mavsdk_check.py
git commit -m "spike: verify MAVSDK <-> PX4 SITL connectivity"
```

### Task 4: MAVSDK coroutine inside a Claude SDK in-process tool (THE GATE)

**Files:**
- Create: `spikes/loop_spike.py`

- [ ] **Step 1: Write the spike**

```python
# spikes/loop_spike.py
"""GO/NO-GO: prove a MAVSDK coroutine can be awaited from inside a
Claude Agent SDK in-process @tool, on a single shared event loop.

Requires: sim running + ANTHROPIC_API_KEY in environment.
"""
import asyncio

from mavsdk import System
from claude_agent_sdk import (
    tool,
    create_sdk_mcp_server,
    ClaudeAgentOptions,
    ClaudeSDKClient,
)

# Connected once, on the loop that ClaudeSDKClient will drive.
_drone = System()


@tool("get_altitude", "Get the drone's current relative altitude in meters", {})
async def get_altitude(args):
    async for position in _drone.telemetry.position():
        alt = position.relative_altitude_m
        return {"content": [{"type": "text", "text": f"relative altitude: {alt:.2f} m"}]}
    return {"content": [{"type": "text", "text": "no telemetry"}], "is_error": True}


async def main() -> None:
    await _drone.connect(system_address="udp://:14540")
    async for state in _drone.core.connection_state():
        if state.is_connected:
            break

    server = create_sdk_mcp_server(name="flight", version="0.0.1", tools=[get_altitude])
    options = ClaudeAgentOptions(
        mcp_servers={"flight": server},
        allowed_tools=["mcp__flight__get_altitude"],
        system_prompt="You control a drone. When asked the altitude, call the tool.",
    )

    async with ClaudeSDKClient(options=options) as client:
        await client.query("What is the drone's current altitude?")
        async for message in client.receive_response():
            print(message)

    print("SPIKE B PASS: MAVSDK coroutine ran inside a Claude SDK tool on one loop")


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 2: Run the gate**

Run (sim running, `ANTHROPIC_API_KEY` set): `. .venv/bin/activate && python spikes/loop_spike.py`
Expected: Claude calls `get_altitude`, the tool prints a real altitude, the run ends with `SPIKE B PASS`.

- [ ] **Step 3: Record the verdict**

If it passes: the in-process single-loop design is validated — proceed. If it fails with a cross-loop error: **STOP** and revisit the architecture (options: run MAVSDK in a dedicated thread with its own loop and bridge via `asyncio.run_coroutine_threadsafe`, or run `mavsdk_server` as a separate process and connect explicitly). Capture the exact error in `spikes/RESULT.md`.

- [ ] **Step 4: Commit**

```bash
git add spikes/loop_spike.py spikes/RESULT.md
git commit -m "spike: GO/NO-GO single-loop MAVSDK-in-Claude-tool gate"
```

---

## Milestone 2 — Pure modules (no sim, full TDD)

These are the safety-critical, fully testable units. A sign error here flies the drone into the ground, so they get strict TDD.

### Task 5: Geodetic offset math (`geo.py`)

**Files:**
- Create: `src/squawd/control/geo.py`
- Test: `tests/test_geo.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_geo.py
import math
from squawd.control.geo import GeoPoint, offset_point, horizontal_distance_m

# Zurich-ish reference; values are convention checks, not site-specific.
ORIGIN = GeoPoint(latitude_deg=47.3977, longitude_deg=8.5456, absolute_altitude_m=500.0)


def test_north_offset_increases_latitude():
    # 1 degree of latitude ~= 111319.49 m
    p = offset_point(ORIGIN, north_m=111319.49, east_m=0.0, up_m=0.0)
    assert math.isclose(p.latitude_deg, ORIGIN.latitude_deg + 1.0, abs_tol=1e-3)
    assert math.isclose(p.longitude_deg, ORIGIN.longitude_deg, abs_tol=1e-9)


def test_south_offset_decreases_latitude():
    p = offset_point(ORIGIN, north_m=-50.0, east_m=0.0, up_m=0.0)
    assert p.latitude_deg < ORIGIN.latitude_deg


def test_east_offset_increases_longitude():
    p = offset_point(ORIGIN, north_m=0.0, east_m=100.0, up_m=0.0)
    assert p.longitude_deg > ORIGIN.longitude_deg


def test_up_increases_absolute_altitude():
    p = offset_point(ORIGIN, north_m=0.0, east_m=0.0, up_m=10.0)
    assert math.isclose(p.absolute_altitude_m, 510.0, abs_tol=1e-9)


def test_distance_roundtrip_50m():
    p = offset_point(ORIGIN, north_m=30.0, east_m=40.0, up_m=0.0)
    assert math.isclose(horizontal_distance_m(ORIGIN, p), 50.0, abs_tol=0.5)
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_geo.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'squawd.control.geo'`.

- [ ] **Step 3: Implement `geo.py`**

```python
# src/squawd/control/geo.py
"""Geodetic offset math. Frame convention (NED-style inputs):
  north_m: + toward geographic north
  east_m:  + toward geographic east
  up_m:    + increases altitude (i.e. -Down)

Local-tangent-plane (equirectangular) approximation about the reference
point. Sub-meter accurate for offsets up to several hundred meters (the v1
envelope). NOT valid near the poles or across the antimeridian.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

_WGS84_A = 6378137.0  # Earth equatorial radius (m)


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    absolute_altitude_m: float


def offset_point(origin: GeoPoint, north_m: float, east_m: float, up_m: float) -> GeoPoint:
    lat_rad = math.radians(origin.latitude_deg)
    dlat_deg = math.degrees(north_m / _WGS84_A)
    dlon_deg = math.degrees(east_m / (_WGS84_A * math.cos(lat_rad)))
    return GeoPoint(
        latitude_deg=origin.latitude_deg + dlat_deg,
        longitude_deg=origin.longitude_deg + dlon_deg,
        absolute_altitude_m=origin.absolute_altitude_m + up_m,
    )


def horizontal_distance_m(a: GeoPoint, b: GeoPoint) -> float:
    lat_rad = math.radians((a.latitude_deg + b.latitude_deg) / 2.0)
    dn = math.radians(b.latitude_deg - a.latitude_deg) * _WGS84_A
    de = math.radians(b.longitude_deg - a.longitude_deg) * _WGS84_A * math.cos(lat_rad)
    return math.hypot(dn, de)
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_geo.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/control/geo.py tests/test_geo.py
git commit -m "feat: geodetic NED offset math with frame conventions"
```

### Task 6: Safety invariants (`safety.py`)

**Files:**
- Create: `src/squawd/control/safety.py`
- Test: `tests/test_safety.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_safety.py
import pytest
from squawd.control.geo import GeoPoint, offset_point
from squawd.control.safety import (
    SafetyGuard, SafetyLimits, SafetyError, DroneSnapshot,
)

HOME = GeoPoint(47.3977, 8.5456, 500.0)
LIMITS = SafetyLimits(
    max_altitude_m=30.0,
    geofence_radius_m=100.0,
    max_goto_distance_m=60.0,
    min_takeoff_altitude_m=2.0,
    max_takeoff_altitude_m=20.0,
)


def flying_snapshot(**over):
    base = dict(
        is_connected=True, is_armed=True, in_air=True, has_position=True,
        flight_mode="HOLD", home=HOME, position=HOME,
    )
    base.update(over)
    return DroneSnapshot(**base)


def test_arm_requires_connection():
    guard = SafetyGuard(LIMITS)
    with pytest.raises(SafetyError):
        guard.check_arm(flying_snapshot(is_connected=False))


def test_takeoff_rejected_when_not_armed():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False, is_armed=False)
    with pytest.raises(SafetyError):
        guard.check_takeoff(10.0, snap)


def test_takeoff_altitude_above_cap_rejected():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False)
    with pytest.raises(SafetyError):
        guard.check_takeoff(999.0, snap)


def test_takeoff_ok_within_limits():
    guard = SafetyGuard(LIMITS)
    snap = flying_snapshot(in_air=False)
    guard.check_takeoff(10.0, snap)  # no raise


def test_goto_requires_position_fix():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 10.0)
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot(has_position=False))


def test_goto_outside_geofence_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 200.0, 0.0, 10.0)  # 200m > 100m radius
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot())


def test_goto_exceeding_per_command_distance_rejected():
    guard = SafetyGuard(LIMITS)
    # within geofence but >60m from current position
    far = offset_point(HOME, 80.0, 0.0, 10.0)
    snap = flying_snapshot(position=HOME)
    with pytest.raises(SafetyError):
        guard.check_goto(far, snap)


def test_goto_above_altitude_cap_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 50.0)  # 50m > 30m cap
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot())


def test_goto_during_failsafe_rejected():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 0.0, 10.0)
    with pytest.raises(SafetyError):
        guard.check_goto(target, flying_snapshot(flight_mode="RETURN_TO_LAUNCH"))


def test_goto_ok_within_all_limits():
    guard = SafetyGuard(LIMITS)
    target = offset_point(HOME, 10.0, 10.0, 10.0)
    guard.check_goto(target, flying_snapshot())  # no raise
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_safety.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'squawd.control.safety'`.

- [ ] **Step 3: Implement `safety.py`**

```python
# src/squawd/control/safety.py
"""Non-bypassable safety supervisor. Pure, LLM-agnostic, unit-tested.
The prompt is NEVER a safety boundary; these invariants are.
"""
from __future__ import annotations

from dataclasses import dataclass

from .geo import GeoPoint, horizontal_distance_m

# Flight modes where the autopilot is in control and new goto commands
# must be refused.
_AUTOPILOT_CONTROLLED = {"RETURN_TO_LAUNCH", "LAND"}


@dataclass(frozen=True)
class SafetyLimits:
    max_altitude_m: float          # max altitude above home
    geofence_radius_m: float       # max horizontal distance from home
    max_goto_distance_m: float     # max distance for a single goto
    min_takeoff_altitude_m: float
    max_takeoff_altitude_m: float


@dataclass(frozen=True)
class DroneSnapshot:
    is_connected: bool
    is_armed: bool
    in_air: bool
    has_position: bool
    flight_mode: str
    home: GeoPoint | None
    position: GeoPoint | None


class SafetyError(Exception):
    """Raised when a command would violate a safety invariant."""


class SafetyGuard:
    def __init__(self, limits: SafetyLimits) -> None:
        self._limits = limits

    def check_arm(self, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot arm: not connected to the vehicle")

    def check_takeoff(self, altitude_m: float, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot take off: not connected")
        if not snap.is_armed:
            raise SafetyError("cannot take off: not armed")
        if snap.in_air:
            raise SafetyError("cannot take off: already in the air")
        if not snap.has_position:
            raise SafetyError("cannot take off: no position fix")
        if altitude_m < self._limits.min_takeoff_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m below minimum "
                f"{self._limits.min_takeoff_altitude_m}m"
            )
        if altitude_m > self._limits.max_takeoff_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m above maximum "
                f"{self._limits.max_takeoff_altitude_m}m"
            )
        if altitude_m > self._limits.max_altitude_m:
            raise SafetyError(
                f"takeoff altitude {altitude_m}m above altitude cap "
                f"{self._limits.max_altitude_m}m"
            )

    def check_goto(self, target: GeoPoint, snap: DroneSnapshot) -> None:
        if not snap.is_connected:
            raise SafetyError("cannot move: not connected")
        if not snap.in_air:
            raise SafetyError("cannot move: not in the air")
        if not snap.has_position or snap.position is None or snap.home is None:
            raise SafetyError("cannot move: no position fix")
        if snap.flight_mode in _AUTOPILOT_CONTROLLED:
            raise SafetyError(
                f"cannot move: autopilot is in control ({snap.flight_mode})"
            )

        dist_from_home = horizontal_distance_m(snap.home, target)
        if dist_from_home > self._limits.geofence_radius_m:
            raise SafetyError(
                f"target {dist_from_home:.0f}m from home exceeds geofence "
                f"radius {self._limits.geofence_radius_m}m"
            )

        dist_from_here = horizontal_distance_m(snap.position, target)
        if dist_from_here > self._limits.max_goto_distance_m:
            raise SafetyError(
                f"move of {dist_from_here:.0f}m exceeds per-command limit "
                f"{self._limits.max_goto_distance_m}m"
            )

        alt_above_home = target.absolute_altitude_m - snap.home.absolute_altitude_m
        if alt_above_home > self._limits.max_altitude_m:
            raise SafetyError(
                f"target altitude {alt_above_home:.0f}m above home exceeds cap "
                f"{self._limits.max_altitude_m}m"
            )
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_safety.py -v`
Expected: 10 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/control/safety.py tests/test_safety.py
git commit -m "feat: non-bypassable safety invariants"
```

### Task 7: Configuration (`config.py`)

**Files:**
- Create: `src/squawd/config.py`
- Test: `tests/test_config.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_config.py
from squawd.config import load_config, Config
from squawd.control.safety import SafetyLimits


def test_defaults_are_conservative(monkeypatch):
    monkeypatch.delenv("SQUAWD_MAX_ALTITUDE_M", raising=False)
    cfg = load_config()
    assert isinstance(cfg, Config)
    assert isinstance(cfg.limits, SafetyLimits)
    # Fail-closed: defaults are tight.
    assert cfg.limits.max_altitude_m <= 30.0
    assert cfg.limits.geofence_radius_m <= 100.0


def test_env_overrides_limit(monkeypatch):
    monkeypatch.setenv("SQUAWD_MAX_ALTITUDE_M", "15")
    cfg = load_config()
    assert cfg.limits.max_altitude_m == 15.0


def test_connection_url_default(monkeypatch):
    monkeypatch.delenv("SQUAWD_CONNECTION_URL", raising=False)
    cfg = load_config()
    assert cfg.connection_url == "udp://:14540"
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_config.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'squawd.config'`.

- [ ] **Step 3: Implement `config.py`**

```python
# src/squawd/config.py
"""The single sim-vs-real seam. All limits fail closed (conservative
defaults if unset). sim->hardware should be a config + connection change.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from squawd.control.safety import SafetyLimits


@dataclass(frozen=True)
class Config:
    connection_url: str
    model: str
    limits: SafetyLimits
    telemetry_rate_hz: float


def _envf(name: str, default: float) -> float:
    raw = os.environ.get(name)
    return float(raw) if raw is not None else default


def load_config() -> Config:
    return Config(
        connection_url=os.environ.get("SQUAWD_CONNECTION_URL", "udp://:14540"),
        model=os.environ.get("SQUAWD_MODEL", "claude-opus-4-8"),
        telemetry_rate_hz=_envf("SQUAWD_TELEMETRY_RATE_HZ", 4.0),
        limits=SafetyLimits(
            max_altitude_m=_envf("SQUAWD_MAX_ALTITUDE_M", 30.0),
            geofence_radius_m=_envf("SQUAWD_GEOFENCE_RADIUS_M", 100.0),
            max_goto_distance_m=_envf("SQUAWD_MAX_GOTO_DISTANCE_M", 60.0),
            min_takeoff_altitude_m=_envf("SQUAWD_MIN_TAKEOFF_ALT_M", 2.0),
            max_takeoff_altitude_m=_envf("SQUAWD_MAX_TAKEOFF_ALT_M", 20.0),
        ),
    )
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_config.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/config.py tests/test_config.py
git commit -m "feat: fail-closed configuration seam"
```

---

## Milestone 3 — Control layer

### Task 8: Authoritative `StateStore`

**Files:**
- Create: `src/squawd/control/state.py`
- Test: `tests/test_state.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_state.py
from squawd.control.state import StateStore
from squawd.control.geo import GeoPoint
from squawd.control.safety import DroneSnapshot


def test_initial_state_is_disconnected():
    store = StateStore()
    snap = store.snapshot()
    assert isinstance(snap, DroneSnapshot)
    assert snap.is_connected is False
    assert snap.has_position is False


def test_updates_are_reflected_in_snapshot():
    store = StateStore()
    store.set_connection(True)
    store.set_armed(True)
    store.set_in_air(True)
    store.set_flight_mode("HOLD")
    store.set_home(GeoPoint(1.0, 2.0, 100.0))
    store.set_position(GeoPoint(1.0, 2.0, 110.0))
    snap = store.snapshot()
    assert snap.is_connected and snap.is_armed and snap.in_air
    assert snap.flight_mode == "HOLD"
    assert snap.has_position is True
    assert snap.position.absolute_altitude_m == 110.0


def test_battery_and_link_health_tracked():
    store = StateStore()
    store.set_battery(0.87)
    store.mark_telemetry_seen(timestamp=123.0)
    assert store.battery_remaining == 0.87
    assert store.last_telemetry_ts == 123.0
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_state.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `state.py`**

```python
# src/squawd/control/state.py
"""Single authoritative drone state, fed by the telemetry task. The LLM is
never the source of truth; every safety check and status report reads here.
"""
from __future__ import annotations

from squawd.control.geo import GeoPoint
from squawd.control.safety import DroneSnapshot


class StateStore:
    def __init__(self) -> None:
        self._connected = False
        self._armed = False
        self._in_air = False
        self._flight_mode = "UNKNOWN"
        self._home: GeoPoint | None = None
        self._position: GeoPoint | None = None
        self.battery_remaining: float | None = None
        self.last_telemetry_ts: float | None = None

    def set_connection(self, value: bool) -> None:
        self._connected = value

    def set_armed(self, value: bool) -> None:
        self._armed = value

    def set_in_air(self, value: bool) -> None:
        self._in_air = value

    def set_flight_mode(self, mode: str) -> None:
        self._flight_mode = mode

    def set_home(self, point: GeoPoint) -> None:
        self._home = point

    def set_position(self, point: GeoPoint) -> None:
        self._position = point

    def set_battery(self, remaining: float) -> None:
        self.battery_remaining = remaining

    def mark_telemetry_seen(self, timestamp: float) -> None:
        self.last_telemetry_ts = timestamp

    @property
    def flight_mode(self) -> str:
        return self._flight_mode

    @property
    def position(self) -> GeoPoint | None:
        return self._position

    @property
    def home(self) -> GeoPoint | None:
        return self._home

    def snapshot(self) -> DroneSnapshot:
        return DroneSnapshot(
            is_connected=self._connected,
            is_armed=self._armed,
            in_air=self._in_air,
            has_position=self._position is not None,
            flight_mode=self._flight_mode,
            home=self._home,
            position=self._position,
        )
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_state.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/control/state.py tests/test_state.py
git commit -m "feat: authoritative StateStore"
```

### Task 9: Telemetry background task

**Files:**
- Create: `src/squawd/control/telemetry.py`
- Test: `tests/test_telemetry.py`

> Telemetry is fed by MAVSDK async-generator streams. To keep this unit-testable without the sim, `run_telemetry` takes the drone object via duck-typed accessors; the test passes a fake exposing async generators.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_telemetry.py
import asyncio
from squawd.control.state import StateStore
from squawd.control.telemetry import update_position_stream


class _FakePosition:
    def __init__(self, lat, lon, abs_alt):
        self.latitude_deg = lat
        self.longitude_deg = lon
        self.absolute_altitude_m = abs_alt
        self.relative_altitude_m = abs_alt - 500.0


async def _one_position():
    yield _FakePosition(47.0, 8.0, 510.0)


async def test_position_stream_updates_store():
    store = StateStore()
    await update_position_stream(_one_position(), store)
    assert store.position is not None
    assert store.position.absolute_altitude_m == 510.0
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_telemetry.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `telemetry.py`**

```python
# src/squawd/control/telemetry.py
"""Background tasks draining MAVSDK telemetry streams into the StateStore.
Each stream is its own coroutine; run them with asyncio.gather in app.py.
No blocking work here — these only update the store.
"""
from __future__ import annotations

import asyncio
import time
from typing import AsyncIterator

from squawd.control.geo import GeoPoint
from squawd.control.state import StateStore


async def update_position_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for position in stream:
        store.set_position(
            GeoPoint(
                latitude_deg=position.latitude_deg,
                longitude_deg=position.longitude_deg,
                absolute_altitude_m=position.absolute_altitude_m,
            )
        )
        store.mark_telemetry_seen(time.monotonic())


async def update_flight_mode_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for mode in stream:
        store.set_flight_mode(str(mode))


async def update_armed_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for armed in stream:
        store.set_armed(bool(armed))


async def update_in_air_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for in_air in stream:
        store.set_in_air(bool(in_air))


async def update_battery_stream(stream: AsyncIterator, store: StateStore) -> None:
    async for battery in stream:
        store.set_battery(float(battery.remaining_percent))


def start_telemetry(drone, store: StateStore) -> list[asyncio.Task]:
    """Spawn one task per telemetry stream. Returns the tasks so the caller
    can cancel them on shutdown."""
    return [
        asyncio.create_task(update_position_stream(drone.telemetry.position(), store)),
        asyncio.create_task(update_flight_mode_stream(drone.telemetry.flight_mode(), store)),
        asyncio.create_task(update_armed_stream(drone.telemetry.armed(), store)),
        asyncio.create_task(update_in_air_stream(drone.telemetry.in_air(), store)),
        asyncio.create_task(update_battery_stream(drone.telemetry.battery(), store)),
    ]
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_telemetry.py -v`
Expected: 1 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/control/telemetry.py tests/test_telemetry.py
git commit -m "feat: telemetry streams -> StateStore"
```

### Task 10: `DroneController` (MAVSDK, fire-and-monitor)

**Files:**
- Create: `src/squawd/control/controller.py`

> Integration-tested against SITL (no TDD-failing-test; it needs the sim). Actions issue the command and return quickly; monitoring of completion is the StateStore's job. Every action has a timeout.

- [ ] **Step 1: Implement `controller.py`**

```python
# src/squawd/control/controller.py
"""Thin async wrapper over MAVSDK actions. Fire-and-monitor: each method
issues the command with a timeout and returns; it does NOT block until the
maneuver completes. This is the code that transfers to real PX4 hardware.
"""
from __future__ import annotations

import asyncio

from mavsdk import System

from squawd.control.geo import GeoPoint

_ACTION_TIMEOUT_S = 10.0


class ControllerError(Exception):
    """A MAVSDK action failed or timed out."""


class DroneController:
    def __init__(self, drone: System) -> None:
        self._drone = drone

    async def connect(self, system_address: str) -> None:
        await self._drone.connect(system_address=system_address)
        async for state in self._drone.core.connection_state():
            if state.is_connected:
                return

    async def _with_timeout(self, coro, what: str):
        try:
            return await asyncio.wait_for(coro, timeout=_ACTION_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            raise ControllerError(f"{what} timed out after {_ACTION_TIMEOUT_S}s") from exc
        except Exception as exc:  # MAVSDK ActionError etc.
            raise ControllerError(f"{what} failed: {exc}") from exc

    async def arm(self) -> None:
        await self._with_timeout(self._drone.action.arm(), "arm")

    async def disarm(self) -> None:
        await self._with_timeout(self._drone.action.disarm(), "disarm")

    async def takeoff(self, altitude_m: float) -> None:
        await self._with_timeout(
            self._drone.action.set_takeoff_altitude(altitude_m), "set takeoff altitude"
        )
        await self._with_timeout(self._drone.action.takeoff(), "takeoff")

    async def land(self) -> None:
        await self._with_timeout(self._drone.action.land(), "land")

    async def return_to_launch(self) -> None:
        await self._with_timeout(self._drone.action.return_to_launch(), "return to launch")

    async def hold(self) -> None:
        await self._with_timeout(self._drone.action.hold(), "hold")

    async def goto(self, target: GeoPoint, yaw_deg: float = float("nan")) -> None:
        await self._with_timeout(
            self._drone.action.goto_location(
                target.latitude_deg,
                target.longitude_deg,
                target.absolute_altitude_m,
                yaw_deg,
            ),
            "goto",
        )
```

- [ ] **Step 2: Manual integration check against SITL**

With the sim running, in a Python REPL (`. .venv/bin/activate && python`):
```python
import asyncio
from mavsdk import System
from squawd.control.controller import DroneController

async def go():
    c = DroneController(System())
    await c.connect("udp://:14540")
    await c.arm(); await c.takeoff(10.0)
    await asyncio.sleep(8)
    await c.land()

asyncio.run(go())
```
Expected: the quad arms, climbs to ~10m in Gazebo, then lands. No `ControllerError`.

- [ ] **Step 3: Commit**

```bash
git add src/squawd/control/controller.py
git commit -m "feat: fire-and-monitor DroneController over MAVSDK"
```

### Task 11: `CommandExecutor` (the portable command boundary)

**Files:**
- Create: `src/squawd/control/executor.py`
- Test: `tests/test_executor.py`

> This is the genuine swap-seam. Plain Python, typed args, structured `CommandResult`. The agent's `@tool`s are thin adapters over it. Unit-tested with a fake controller so safety logic is verified without the sim.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_executor.py
from squawd.control.executor import CommandExecutor, CommandResult
from squawd.control.state import StateStore
from squawd.control.safety import SafetyGuard, SafetyLimits
from squawd.control.geo import GeoPoint

LIMITS = SafetyLimits(30.0, 100.0, 60.0, 2.0, 20.0)
HOME = GeoPoint(47.3977, 8.5456, 500.0)


class FakeController:
    def __init__(self):
        self.calls = []

    async def arm(self): self.calls.append("arm")
    async def takeoff(self, alt): self.calls.append(("takeoff", alt))
    async def land(self): self.calls.append("land")
    async def goto(self, target, yaw_deg=float("nan")): self.calls.append(("goto", target))


def armed_store(in_air=False):
    s = StateStore()
    s.set_connection(True)
    s.set_armed(True)
    s.set_in_air(in_air)
    s.set_flight_mode("HOLD")
    s.set_home(HOME)
    s.set_position(HOME)
    return s


async def test_takeoff_blocked_by_safety_returns_error_result():
    ex = CommandExecutor(FakeController(), armed_store(in_air=False), SafetyGuard(LIMITS))
    result = await ex.takeoff(999.0)
    assert isinstance(result, CommandResult)
    assert result.ok is False
    assert "cap" in result.message or "maximum" in result.message


async def test_takeoff_ok_calls_controller():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=False), SafetyGuard(LIMITS))
    result = await ex.takeoff(10.0)
    assert result.ok is True
    assert ("takeoff", 10.0) in fake.calls


async def test_goto_relative_translates_and_calls_controller():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=True), SafetyGuard(LIMITS))
    result = await ex.goto_relative(north_m=10.0, east_m=0.0, up_m=0.0)
    assert result.ok is True
    assert fake.calls and fake.calls[-1][0] == "goto"


async def test_goto_relative_outside_geofence_blocked():
    fake = FakeController()
    ex = CommandExecutor(fake, armed_store(in_air=True), SafetyGuard(LIMITS))
    result = await ex.goto_relative(north_m=500.0, east_m=0.0, up_m=0.0)
    assert result.ok is False
    assert not any(c[0] == "goto" for c in fake.calls if isinstance(c, tuple))
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_executor.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `executor.py`**

```python
# src/squawd/control/executor.py
"""Portable command boundary. SDK-agnostic. Combines controller + safety +
state into typed commands returning structured results. Any agent (Claude,
other LLM, scripted) targets this interface.
"""
from __future__ import annotations

from dataclasses import dataclass

from squawd.control.controller import ControllerError
from squawd.control.geo import GeoPoint, offset_point
from squawd.control.safety import SafetyError, SafetyGuard
from squawd.control.state import StateStore


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    message: str


class CommandExecutor:
    def __init__(self, controller, state: StateStore, guard: SafetyGuard) -> None:
        self._c = controller
        self._state = state
        self._guard = guard

    async def _run(self, check, action, success_msg: str) -> CommandResult:
        try:
            if check is not None:
                check()
            await action()
            return CommandResult(True, success_msg)
        except SafetyError as exc:
            return CommandResult(False, f"refused: {exc}")
        except ControllerError as exc:
            return CommandResult(False, f"command failed: {exc}")

    async def arm(self) -> CommandResult:
        snap = self._state.snapshot()
        return await self._run(
            lambda: self._guard.check_arm(snap), self._c.arm, "armed"
        )

    async def takeoff(self, altitude_m: float) -> CommandResult:
        snap = self._state.snapshot()
        return await self._run(
            lambda: self._guard.check_takeoff(altitude_m, snap),
            lambda: self._c.takeoff(altitude_m),
            f"taking off to {altitude_m:.0f}m (climbing)",
        )

    async def land(self) -> CommandResult:
        return await self._run(None, self._c.land, "landing")

    async def return_to_launch(self) -> CommandResult:
        return await self._run(None, self._c.return_to_launch, "returning to launch")

    async def hold(self) -> CommandResult:
        return await self._run(None, self._c.hold, "holding position")

    async def goto_relative(self, north_m: float, east_m: float, up_m: float) -> CommandResult:
        snap = self._state.snapshot()
        if snap.position is None:
            return CommandResult(False, "refused: no position fix")
        target = offset_point(snap.position, north_m, east_m, up_m)
        return await self._run(
            lambda: self._guard.check_goto(target, snap),
            lambda: self._c.goto(target),
            f"moving N{north_m:.0f} E{east_m:.0f} U{up_m:.0f} (in progress)",
        )

    def status(self) -> CommandResult:
        snap = self._state.snapshot()
        bat = self._state.battery_remaining
        bat_s = f"{bat * 100:.0f}%" if bat is not None else "unknown"
        return CommandResult(
            True,
            f"connected={snap.is_connected} armed={snap.is_armed} "
            f"in_air={snap.in_air} mode={snap.flight_mode} battery={bat_s}",
        )
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_executor.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/control/executor.py tests/test_executor.py
git commit -m "feat: CommandExecutor portable command boundary"
```

---

## Milestone 4 — Perception layer

### Task 12: `PerceptionProvider` interface + `PerceptionStore`

**Files:**
- Create: `src/squawd/perception/provider.py`
- Create: `src/squawd/perception/store.py`
- Test: `tests/test_perception_store.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_perception_store.py
from squawd.perception.provider import PerceptionSnapshot, Obstacle
from squawd.perception.store import PerceptionStore


def test_store_starts_empty():
    store = PerceptionStore()
    assert store.latest() is None


def test_store_returns_last_snapshot_and_summary():
    store = PerceptionStore()
    snap = PerceptionSnapshot(
        timestamp=1.0,
        jpeg_frame=b"\xff\xd8fake",
        obstacles=[Obstacle(direction="ahead", distance_m=4.0),
                   Obstacle(direction="left", distance_m=9.0)],
    )
    store.update(snap)
    assert store.latest() is snap
    # nearest obstacle leads the summary
    assert "4" in store.surroundings_summary()
    assert "ahead" in store.surroundings_summary()


def test_summary_when_clear():
    store = PerceptionStore()
    store.update(PerceptionSnapshot(timestamp=1.0, jpeg_frame=None, obstacles=[]))
    assert "clear" in store.surroundings_summary().lower()
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_perception_store.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `provider.py`**

```python
# src/squawd/perception/provider.py
"""Perception modularity contract. The agent and control layers only ever
see PerceptionSnapshot — never the sensor source. Swap GazeboPerception for
a ROS2 / real-sensor provider later without touching anything above.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Obstacle:
    direction: str       # e.g. "ahead", "left", "ahead-left"
    distance_m: float


@dataclass(frozen=True)
class PerceptionSnapshot:
    timestamp: float
    jpeg_frame: bytes | None          # RGB camera frame for the agent's vision
    obstacles: list[Obstacle] = field(default_factory=list)


class PerceptionProvider(ABC):
    @abstractmethod
    async def start(self) -> None:
        """Begin streaming sensor data into the bound store."""

    @abstractmethod
    async def stop(self) -> None:
        """Stop streaming and release sensor resources."""
```

- [ ] **Step 4: Implement `store.py`**

```python
# src/squawd/perception/store.py
"""Authoritative latest perception snapshot. Mirrors StateStore."""
from __future__ import annotations

from squawd.perception.provider import PerceptionSnapshot


class PerceptionStore:
    def __init__(self) -> None:
        self._latest: PerceptionSnapshot | None = None

    def update(self, snapshot: PerceptionSnapshot) -> None:
        self._latest = snapshot

    def latest(self) -> PerceptionSnapshot | None:
        return self._latest

    def surroundings_summary(self) -> str:
        if self._latest is None:
            return "no perception data yet"
        if not self._latest.obstacles:
            return "surroundings clear"
        nearest = min(self._latest.obstacles, key=lambda o: o.distance_m)
        return f"nearest obstacle: {nearest.distance_m:.0f}m {nearest.direction}"
```

- [ ] **Step 5: Run to verify pass**

Run: `pytest tests/test_perception_store.py -v`
Expected: 3 passed.

- [ ] **Step 6: Commit**

```bash
git add src/squawd/perception/provider.py src/squawd/perception/store.py tests/test_perception_store.py
git commit -m "feat: perception interface + store"
```

### Task 13: `GazeboPerception` (v1 sensor reader)

**Files:**
- Create: `src/squawd/perception/gazebo_perception.py`

> Reads the depth/camera sensor the `gz_x500_depth` model publishes on Gazebo transport topics. Gazebo's Python transport API (`gz.transport`) callbacks are synchronous; we marshal frames into the async store via `asyncio.to_thread` / a thread-safe handoff so we never block the main loop (per spec §5.1/§7). No failing-test TDD — verified against the running sim.

- [ ] **Step 1: Implement `gazebo_perception.py`**

```python
# src/squawd/perception/gazebo_perception.py
"""v1 perception: subscribe to the sim's RGB + depth camera, derive a coarse
nearest-obstacle reading and the latest JPEG frame, push PerceptionSnapshots
into the PerceptionStore. Easy-sensors-first; swappable later.

Implementation note: Gazebo transport callbacks run on Gazebo's own threads.
We convert + enqueue there (cheap), and a small asyncio poller drains the
queue into the store, so the main event loop is never blocked.
"""
from __future__ import annotations

import asyncio
import time
from queue import Empty, Queue

from squawd.perception.provider import (
    Obstacle, PerceptionProvider, PerceptionSnapshot,
)

# Gazebo transport + msgs. Import names per Gazebo Harmonic Python bindings.
from gz.transport13 import Node           # type: ignore
from gz.msgs10.image_pb2 import Image      # type: ignore


def _depth_to_obstacles(depth_image) -> list[Obstacle]:
    """Coarse nearest-obstacle estimate from a depth frame: sample the
    center column, report the closest valid return as 'ahead'. Refined
    later (multi-sector, lidar)."""
    # Placeholder-free minimal logic: caller passes pre-decoded min distances.
    raise NotImplementedError  # replaced in Step 2 with the concrete decode


class GazeboPerception(PerceptionProvider):
    def __init__(self, store, rgb_topic: str, depth_topic: str) -> None:
        self._store = store
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._node = Node()
        self._queue: "Queue[tuple[bytes | None, list[Obstacle]]]" = Queue(maxsize=4)
        self._latest_jpeg: bytes | None = None
        self._latest_obstacles: list[Obstacle] = []
        self._poller: asyncio.Task | None = None
        self._running = False

    def _on_rgb(self, msg: Image) -> None:
        # Store raw bytes; JPEG-encode lazily in the poller thread to keep
        # the Gazebo callback cheap.
        self._latest_jpeg = bytes(msg.data)

    def _on_depth(self, msg) -> None:
        self._latest_obstacles = _depth_to_obstacles(msg)

    async def start(self) -> None:
        self._node.subscribe(Image, self._rgb_topic, self._on_rgb)
        self._node.subscribe(Image, self._depth_topic, self._on_depth)
        self._running = True
        self._poller = asyncio.create_task(self._poll())

    async def _poll(self) -> None:
        while self._running:
            snap = PerceptionSnapshot(
                timestamp=time.monotonic(),
                jpeg_frame=self._latest_jpeg,
                obstacles=list(self._latest_obstacles),
            )
            self._store.update(snap)
            await asyncio.sleep(0.25)  # 4 Hz; perception need not be fast

    async def stop(self) -> None:
        self._running = False
        if self._poller is not None:
            self._poller.cancel()
```

- [ ] **Step 2: Replace the depth decode with concrete logic**

Replace the `_depth_to_obstacles` body with a real decode of the depth image format the `gz_x500_depth` model publishes (typically `R_FLOAT32` depth in meters). Determine the exact topic names and encoding first:

Run (sim running): `gz topic -l | grep -Ei 'depth|image|camera'`
Then inspect one message: `gz topic -e -t <depth_topic> -n 1 | head -40`

Implement the decode to: read the depth buffer as float32, take a center patch, ignore non-finite/zero returns, and emit a single `Obstacle("ahead", min_distance)` if the closest return is under a threshold (e.g. 15 m), else `[]`. Wire `rgb_topic`/`depth_topic` from the inspected names.

- [ ] **Step 3: Verify against the sim**

Write a tiny scratch runner (not committed) that constructs `GazeboPerception` with a `PerceptionStore`, calls `start()`, sleeps 2s in an asyncio loop, and prints `store.surroundings_summary()` and `len(store.latest().jpeg_frame)`.
Expected: a non-empty JPEG length and a plausible summary ("surroundings clear" when nothing is in front).

- [ ] **Step 4: Commit**

```bash
git add src/squawd/perception/gazebo_perception.py
git commit -m "feat: Gazebo RGB+depth perception provider"
```

---

## Milestone 5 — Agent

### Task 14: System prompt

**Files:**
- Create: `src/squawd/agent/prompts.py`
- Test: `tests/test_prompts.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_prompts.py
from squawd.agent.prompts import SYSTEM_PROMPT


def test_prompt_sets_role_and_safety_framing():
    p = SYSTEM_PROMPT.lower()
    assert "drone" in p
    assert "tool" in p           # must act via tools, not narration
    assert "status" in p         # encourage grounding in real state
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_prompts.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `prompts.py`**

```python
# src/squawd/agent/prompts.py
"""System prompt for the drone-piloting agent."""

SYSTEM_PROMPT = """\
You are the pilot of a simulated quadcopter drone. The user talks to you in
plain language; you fly the drone by calling the provided tools. You never
pretend to fly — every action happens through a tool call.

Rules:
- Take off before trying to move; you cannot move on the ground.
- Movement is relative to the drone (e.g. "50m north" -> goto_relative).
- The flight controller enforces hard safety limits (altitude cap, geofence,
  collision prevention). If a tool returns refused/failed, tell the user the
  reason plainly and suggest a legal alternative. Do not try to circumvent it.
- The tool results are the ground truth about the drone's state, not your
  memory. When unsure, call get_status before acting.
- For questions about the environment ("what do you see?", "anything ahead?"),
  call look or scan_surroundings and answer from what they return.
- Be concise and confirm what you did, including in-progress maneuvers.
"""
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_prompts.py -v`
Expected: 1 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/agent/prompts.py tests/test_prompts.py
git commit -m "feat: drone pilot system prompt"
```

### Task 15: Tool adapters

**Files:**
- Create: `src/squawd/agent/tools.py`
- Test: `tests/test_tools.py`

> Tools are thin adapters: call `CommandExecutor`/`PerceptionStore`, format the result, and **return `is_error: True` rather than raising** (an uncaught exception kills the SDK query loop, per spec §5.4). `build_flight_tools(executor, perception_store)` returns the SDK server so dependencies are injected (testable).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_tools.py
from squawd.agent.tools import make_takeoff_tool, make_status_tool
from squawd.control.executor import CommandResult


class FakeExecutor:
    def __init__(self, result): self._result = result; self.called = False
    async def takeoff(self, altitude_m): self.called = True; return self._result
    def status(self): return CommandResult(True, "all good")


async def test_takeoff_tool_reports_success():
    ex = FakeExecutor(CommandResult(True, "taking off to 10m (climbing)"))
    tool_fn = make_takeoff_tool(ex)
    out = await tool_fn({"altitude_m": 10.0})
    assert ex.called
    assert out.get("is_error") in (None, False)
    assert "climbing" in out["content"][0]["text"]


async def test_takeoff_tool_reports_refusal_as_error():
    ex = FakeExecutor(CommandResult(False, "refused: not armed"))
    tool_fn = make_takeoff_tool(ex)
    out = await tool_fn({"altitude_m": 10.0})
    assert out["is_error"] is True
    assert "not armed" in out["content"][0]["text"]


async def test_status_tool():
    ex = FakeExecutor(CommandResult(True, "x"))
    out = await make_status_tool(ex)({})
    assert "all good" in out["content"][0]["text"]
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_tools.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `tools.py`**

```python
# src/squawd/agent/tools.py
"""Claude Agent SDK in-process tool adapters over CommandExecutor and
PerceptionStore. Thin: translate, format, and surface errors as is_error.
"""
from __future__ import annotations

import base64

from claude_agent_sdk import tool, create_sdk_mcp_server

from squawd.control.executor import CommandExecutor, CommandResult
from squawd.perception.store import PerceptionStore


def _text(msg: str, is_error: bool = False) -> dict:
    out = {"content": [{"type": "text", "text": msg}]}
    if is_error:
        out["is_error"] = True
    return out


def _from_result(result: CommandResult) -> dict:
    return _text(result.message, is_error=not result.ok)


# Factory functions return bare async handlers (testable without the SDK).
def make_takeoff_tool(executor: CommandExecutor):
    async def handler(args):
        return _from_result(await executor.takeoff(float(args["altitude_m"])))
    return handler


def make_status_tool(executor: CommandExecutor):
    async def handler(args):
        return _from_result(executor.status())
    return handler


def build_flight_server(executor: CommandExecutor, perception: PerceptionStore):
    """Register all tools with the SDK and return the in-process MCP server."""

    @tool("arm", "Arm the drone motors", {})
    async def arm(args):
        return _from_result(await executor.arm())

    @tool("takeoff", "Take off and climb to the given altitude in meters", {"altitude_m": float})
    async def takeoff(args):
        return _from_result(await executor.takeoff(float(args["altitude_m"])))

    @tool("land", "Land the drone at the current position", {})
    async def land(args):
        return _from_result(await executor.land())

    @tool("return_to_launch", "Fly back to the launch point and land", {})
    async def rtl(args):
        return _from_result(await executor.return_to_launch())

    @tool("hold", "Stop and hover in place", {})
    async def hold(args):
        return _from_result(await executor.hold())

    @tool(
        "goto_relative",
        "Move relative to the drone in meters: north/east/up (negatives for south/west/down)",
        {"north_m": float, "east_m": float, "up_m": float},
    )
    async def goto_relative(args):
        return _from_result(await executor.goto_relative(
            float(args["north_m"]), float(args["east_m"]), float(args["up_m"]),
        ))

    @tool("get_status", "Report the drone's current state", {})
    async def get_status(args):
        return _from_result(executor.status())

    @tool("scan_surroundings", "Report nearby obstacles from the depth sensor", {})
    async def scan_surroundings(args):
        return _text(perception.surroundings_summary())

    @tool("look", "Look through the drone camera and return the current image", {})
    async def look(args):
        snap = perception.latest()
        if snap is None or snap.jpeg_frame is None:
            return _text("no camera image available", is_error=True)
        b64 = base64.b64encode(snap.jpeg_frame).decode("ascii")
        return {
            "content": [
                {"type": "text", "text": perception.surroundings_summary()},
                {"type": "image",
                 "source": {"type": "base64", "media_type": "image/jpeg", "data": b64}},
            ]
        }

    return create_sdk_mcp_server(
        name="flight",
        version="1.0.0",
        tools=[arm, takeoff, land, rtl, hold, goto_relative, get_status,
               scan_surroundings, look],
    )


ALLOWED_TOOLS = [
    "mcp__flight__arm", "mcp__flight__takeoff", "mcp__flight__land",
    "mcp__flight__return_to_launch", "mcp__flight__hold",
    "mcp__flight__goto_relative", "mcp__flight__get_status",
    "mcp__flight__scan_surroundings", "mcp__flight__look",
]
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_tools.py -v`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/agent/tools.py tests/test_tools.py
git commit -m "feat: flight tool adapters (is_error on failure)"
```

### Task 16: Agent wiring (`claude_agent.py`)

**Files:**
- Create: `src/squawd/agent/claude_agent.py`

> Wires the SDK options and exposes a small async interface (`ask`, `interrupt`) the REPL uses. No failing-test TDD (it owns a live SDK client); verified in the end-to-end task.

- [ ] **Step 1: Implement `claude_agent.py`**

```python
# src/squawd/agent/claude_agent.py
"""Claude Agent SDK wiring. Owns the ClaudeSDKClient lifecycle and exposes a
minimal interface to the REPL: ask(text) streams a reply; interrupt() aborts
the current turn (the hard safety abort is separate and bypasses this).
"""
from __future__ import annotations

from typing import AsyncIterator

from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient

from squawd.agent.prompts import SYSTEM_PROMPT
from squawd.agent.tools import build_flight_server, ALLOWED_TOOLS
from squawd.control.executor import CommandExecutor
from squawd.perception.store import PerceptionStore


class DroneAgent:
    def __init__(self, executor: CommandExecutor, perception: PerceptionStore, model: str) -> None:
        server = build_flight_server(executor, perception)
        self._options = ClaudeAgentOptions(
            model=model,
            system_prompt=SYSTEM_PROMPT,
            mcp_servers={"flight": server},
            allowed_tools=ALLOWED_TOOLS,
        )
        self._client: ClaudeSDKClient | None = None

    async def __aenter__(self) -> "DroneAgent":
        self._client = ClaudeSDKClient(options=self._options)
        await self._client.__aenter__()
        return self

    async def __aexit__(self, *exc) -> None:
        assert self._client is not None
        await self._client.__aexit__(*exc)

    async def ask(self, text: str) -> AsyncIterator:
        assert self._client is not None
        await self._client.query(text)
        async for message in self._client.receive_response():
            yield message

    async def interrupt(self) -> None:
        if self._client is not None:
            await self._client.interrupt()
```

- [ ] **Step 2: Commit**

```bash
git add src/squawd/agent/claude_agent.py
git commit -m "feat: DroneAgent SDK wiring"
```

---

## Milestone 6 — Flight log, REPL, wiring, end-to-end

### Task 17: Structured flight log

**Files:**
- Create: `src/squawd/flight_log.py`
- Test: `tests/test_flight_log.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_flight_log.py
import json
from squawd.flight_log import FlightLog


def test_log_writes_jsonl_records(tmp_path):
    path = tmp_path / "flight.jsonl"
    log = FlightLog(str(path))
    log.record("utterance", {"text": "take off"})
    log.record("command_result", {"ok": True, "message": "climbing"})
    lines = path.read_text().strip().splitlines()
    assert len(lines) == 2
    first = json.loads(lines[0])
    assert first["kind"] == "utterance"
    assert first["data"]["text"] == "take off"
    assert "ts" in first
```

- [ ] **Step 2: Run to verify failure**

Run: `pytest tests/test_flight_log.py -v`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `flight_log.py`**

```python
# src/squawd/flight_log.py
"""Append-only JSONL flight record: utterance -> tool call -> safety
decision -> result -> telemetry. The experiment's audit trail.
"""
from __future__ import annotations

import json
import time
from typing import Any


class FlightLog:
    def __init__(self, path: str) -> None:
        self._path = path

    def record(self, kind: str, data: dict[str, Any]) -> None:
        entry = {"ts": time.time(), "kind": kind, "data": data}
        with open(self._path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry) + "\n")
```

- [ ] **Step 4: Run to verify pass**

Run: `pytest tests/test_flight_log.py -v`
Expected: 1 passed.

- [ ] **Step 5: Commit**

```bash
git add src/squawd/flight_log.py tests/test_flight_log.py
git commit -m "feat: structured JSONL flight log"
```

### Task 18: REPL with direct (non-LLM) abort

**Files:**
- Create: `src/squawd/chat/repl.py`

> Async stdin via `loop.run_in_executor(None, input, ...)`. Typing `stop`/`abort`/`land` (or empty line + Ctrl-C handling) triggers a **direct** controller hold/land that bypasses the agent (spec §5.2), and also calls `agent.interrupt()`. Verified end-to-end in Task 20.

- [ ] **Step 1: Implement `repl.py`**

```python
# src/squawd/chat/repl.py
"""Terminal chat loop. Renders agent replies and status; owns the direct,
non-LLM abort path.
"""
from __future__ import annotations

import asyncio

from squawd.agent.claude_agent import DroneAgent
from squawd.control.executor import CommandExecutor
from squawd.flight_log import FlightLog

_ABORT_WORDS = {"stop", "abort", "emergency", "land now"}


async def _read_line(prompt: str) -> str:
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, input, prompt)


def _text_of(message) -> str:
    # Extract printable text from an SDK message; tolerate non-text blocks.
    content = getattr(message, "content", None)
    if not content:
        return ""
    parts = []
    for block in content:
        text = getattr(block, "text", None)
        if text:
            parts.append(text)
    return " ".join(parts)


async def run_repl(agent: DroneAgent, executor: CommandExecutor, log: FlightLog) -> None:
    print("Drone chatbot ready. Type a command, 'status', or 'stop' to abort. Ctrl-D to quit.")
    while True:
        try:
            user = (await _read_line("\nyou> ")).strip()
        except EOFError:
            print("\nshutting down.")
            return

        if not user:
            continue

        if user.lower() in _ABORT_WORDS:
            # DIRECT abort — never routed through the LLM.
            log.record("abort", {"trigger": user})
            await agent.interrupt()
            result = await executor.hold()
            print(f"[ABORT] {result.message}")
            continue

        log.record("utterance", {"text": user})
        print("drone> ", end="", flush=True)
        async for message in agent.ask(user):
            text = _text_of(message)
            if text:
                print(text, end="", flush=True)
        print()
```

- [ ] **Step 2: Commit**

```bash
git add src/squawd/chat/repl.py
git commit -m "feat: REPL with direct non-LLM abort"
```

### Task 19: Application wiring (`app.py`) — single loop owner

**Files:**
- Create: `src/squawd/app.py`

> Owns the one event loop. Order matters: connect the controller, start telemetry, start perception, enter the agent, then run the REPL. On exit, cancel telemetry + stop perception cleanly.

- [ ] **Step 1: Implement `app.py`**

```python
# src/squawd/app.py
"""Entrypoint. Owns the single asyncio loop and wires all layers."""
from __future__ import annotations

import asyncio
import os

from dotenv import load_dotenv
from mavsdk import System

from squawd.agent.claude_agent import DroneAgent
from squawd.chat.repl import run_repl
from squawd.config import load_config
from squawd.control.controller import DroneController
from squawd.control.executor import CommandExecutor
from squawd.control.safety import SafetyGuard
from squawd.control.state import StateStore
from squawd.control.telemetry import start_telemetry
from squawd.flight_log import FlightLog
from squawd.perception.gazebo_perception import GazeboPerception
from squawd.perception.store import PerceptionStore

# Topic names confirmed via `gz topic -l` in Task 13.
_RGB_TOPIC = os.environ.get("SQUAWD_RGB_TOPIC", "/camera")
_DEPTH_TOPIC = os.environ.get("SQUAWD_DEPTH_TOPIC", "/depth_camera")


async def main() -> None:
    load_dotenv()
    cfg = load_config()

    drone = System()
    controller = DroneController(drone)
    print(f"connecting to {cfg.connection_url} ...")
    await controller.connect(cfg.connection_url)
    print("connected.")

    state = StateStore()
    state.set_connection(True)
    telemetry_tasks = start_telemetry(drone, state)

    # Capture home once a position fix arrives.
    for _ in range(40):
        if state.position is not None:
            state.set_home(state.position)
            break
        await asyncio.sleep(0.25)

    perception_store = PerceptionStore()
    perception = GazeboPerception(perception_store, _RGB_TOPIC, _DEPTH_TOPIC)
    await perception.start()

    guard = SafetyGuard(cfg.limits)
    executor = CommandExecutor(controller, state, guard)
    log = FlightLog("flight_logs/session.jsonl")
    os.makedirs("flight_logs", exist_ok=True)

    try:
        async with DroneAgent(executor, perception_store, cfg.model) as agent:
            await run_repl(agent, executor, log)
    finally:
        await perception.stop()
        for task in telemetry_tasks:
            task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
```

- [ ] **Step 2: Commit**

```bash
git add src/squawd/app.py
git commit -m "feat: app entrypoint owning the single event loop"
```

### Task 20: End-to-end verification

**Files:** none (verification only)

- [ ] **Step 1: Full test suite (no sim)**

Run: `. .venv/bin/activate && pytest -q`
Expected: all unit tests pass.

- [ ] **Step 2: Launch the sim**

Run: `PX4_DIR=$HOME/PX4-Autopilot ./scripts/run_sim.sh`
Expected: Gazebo up, `Ready for takeoff!`.

- [ ] **Step 3: Run the app and fly a scripted conversation**

Run (new terminal, `ANTHROPIC_API_KEY` set): `. .venv/bin/activate && python -m squawd.app`
Then type, one at a time:
```
arm
take off to 10 meters
fly 20 meters north
what do you see?
come back and land
```
Expected: drone arms, climbs to ~10m in Gazebo, translates 20m north and moves, `look`/`scan_surroundings` returns an image/summary, RTL + land. Each maneuver is confirmed in chat.

- [ ] **Step 4: Verify safety + abort**

Type: `fly 500 meters north` → expected: agent reports the geofence refusal, drone does not move.
Type: `take off to 10 meters` then immediately `stop` → expected: `[ABORT] holding position`, drone holds, abort logged.

- [ ] **Step 5: Verify the flight log**

Run: `tail -n 20 flight_logs/session.jsonl`
Expected: JSONL records for utterances, abort, and command results.

- [ ] **Step 6: Final commit**

```bash
git add -A
git commit -m "test: end-to-end sim verification notes"
```

---

## Self-Review Notes

- **Spec coverage:** core flight (Tasks 6/10/11/15), perception awareness + visual Q&A (Tasks 12/13/15 `look`/`scan_surroundings`), single-loop (Task 4 gate, Task 19), fire-and-monitor + non-LLM abort (Tasks 10/11/18), authoritative state (Task 8), SafetyGuard invariants (Task 6), CommandExecutor seam (Task 11), `is_error` discipline (Task 15), flight log (Task 17), config seam (Task 7), geofence/collision-prevention reuse (Task 2 sim params — see note below). All spec sections map to a task.
- **Open item carried from spec:** PX4 geofence/collision-prevention params (`GF_*`, `CP_DIST`) should be set on the SITL airframe. Add them to `scripts/run_sim.sh` or a PX4 startup script once topic/param names are confirmed in Task 13; defense-in-depth on top of the app-side SafetyGuard.
- **Deferred (non-goals, intentionally absent):** ROS2, object detection/SLAM, autonomous avoidance, web/voice, mission patterns, second LLM provider.
