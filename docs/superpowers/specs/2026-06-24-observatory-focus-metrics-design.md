# Observatory: focus/expand view + per-drone metrics

**Date:** 2026-06-24
**Status:** Approved design, ready for implementation plan
**Scope:** `agents/observatory/server.py`, `agents/observatory/static/index.html`

## Goal

Make the Swarm Observatory web UI more ergonomic and better looking. Specifically:
click (or otherwise select) a drone and see an enlarged live camera "hero" view
plus the important metrics for that drone.

## Decisions (settled during brainstorming)

- **Interaction model:** focus/expand *in place* (not a side drawer, not map-only).
  One drone is "selected" at a time; selecting it enlarges its camera and shows
  its metrics.
- **Metrics shown:** all four groups — position/motion, flight status,
  battery/health, agent task.
- **Layout:** "Hero-top + metrics rail" (see ASCII below).
- **No new frameworks.** Stays vanilla HTML/CSS/JS inside the single
  `static/index.html`, matching the current implementation.

## Layout

```
┌──────────────────────────────────────────────┐
│ SQUAWD · SWARM OBSERVATORY     3·live  ●     │
├────────────────────────────┬───────────────────┤
│                            │  drone_1  ● ARMED │
│      HERO CAMERA           │  ───────────────  │
│      (selected drone)      │  ALT    12.4 m    │
│                            │  SPD    3.1 m/s   │
│                            │  HDG    074°      │
│                            │  MODE   OFFBOARD  │
│                            │  BATT   78% 15.6V │
│                            │  TASK   survey N  │
├────────────────────────────┴───────────────────┤
│ [d0] [d1●] [d2] [d3] [d4] …  ← click to select  │
├──────────────────────┬──────────────────────────┤
│   MAP (click drones) │  SWARM CHAT + COMMAND     │
└──────────────────────┴──────────────────────────┘
```

- **Hero camera** (left, large): the selected drone's live POV.
- **Metrics rail** (right): selected drone's readout, grouped Position/Motion ·
  Flight status · Battery · Agent task. Header shows `drone_<i>` in that drone's
  accent color + an ARMED/DISARMED badge.
- **Thumbnail strip**: every drone's live mini-feed; click to select. Selected
  thumb gets an accent ring; each thumb shows id + a tiny status dot.
- **Map** (bottom-left): top-down, markers clickable to select; selected marker
  highlighted (e.g. larger / ringed).
- **Chat + command** (bottom-right): behavior unchanged.
- **Default selection:** `drone_0`. Selection state is client-side only.

## Aesthetic

Keep the existing dark mission-control palette (`--bg`, `--panel`, `--ink`,
`--accent`, `--grid`). Refinements only:

- Clear hierarchy: the hero dominates; metrics are glanceable.
- Per-drone accent color carried consistently across thumbnail ring, map marker,
  and metrics header (reuse the existing `COLORS` array / `col(i)` helper).
- Small state badges: ARMED green, DISARMED muted; battery color-coded
  (e.g. green > 50%, amber 20–50%, red < 20% or `warn` set).
- Tighter, consistent spacing and typography. No new fonts.

## Data flow — no new browser connections

The single `/ws` WebSocket already streams *every* drone's JPEG frames (binary:
byte 0 = drone id, rest = JPEG). The hero and all thumbnails render from that
same socket; the hero simply draws the currently-selected id at large size.
Adding the hero therefore costs zero extra bandwidth or connections, and the
browser's ~6-connection-per-host cap stays irrelevant.

Selection is a client-side variable. On each WS frame for drone `i`: update
thumb `i`; if `i === selected`, also update the hero `<img>`.

## Backend changes (`server.py`)

### New subscriptions (per drone `i`, in the existing `for _i in range(N)` loop)

- `/px4_<i>/fmu/out/vehicle_status` → `VehicleStatus`
- `/px4_<i>/fmu/out/battery_status` → `BatteryStatus`

Both use `PX4_QOS` (the default for `bridge.subscribe`).

### New latched logs for agent task/report

Subscribe the observatory to each drone's command inbox and report outbox so the
UI can show "current task" and "last result":

- `/swarm/cmd/drone_<i>` (String, `CHAT_QOS`) → last dispatched task
- `/swarm/report/drone_<i>` (String, `CHAT_QOS`) → last report

Use `TopicLog` (as already done for `/swarm/chat`) and read the most recent
entry, or `bridge.latest(...)` since these are latched String channels. Either
is fine; prefer whichever keeps `server.py` simplest.

### Enum → string mapping

A small module-level dict in `server.py` maps:

- `arming_state`: `2` → armed `True`, else `False` (constant `ARMING_STATE_ARMED = 2`).
- `nav_state` → readable mode string. Cover at least: `MANUAL(0)`,
  `AUTO_MISSION(3)`, `AUTO_LOITER(4) → HOLD`, `OFFBOARD(14)`,
  `AUTO_TAKEOFF(17) → TAKEOFF`, `AUTO_LAND(18) → LAND`. Unknown values →
  `f"#{nav_state}"` so nothing crashes.

### Expanded `/state` per-drone object

Grows from `{id, north, east, alt, cam}` to add:

| field      | source / derivation                                              |
|------------|------------------------------------------------------------------|
| `speed`    | `hypot(vx, vy)` from VehicleLocalPosition, rounded               |
| `vspeed`   | `-vz` (up-positive), rounded (optional; include if cheap)        |
| `heading`  | `degrees(heading)` normalized to 0–360, rounded                  |
| `armed`    | bool from VehicleStatus.arming_state                             |
| `mode`     | nav_state mapped to string                                       |
| `batt_pct` | `round(remaining * 100)` if `remaining >= 0` else `None`         |
| `voltage`  | `round(voltage_v, 1)` if known else `None`                       |
| `warn`     | `BatteryStatus.warning` level (for battery color coding)         |
| `task`     | last `/swarm/cmd/drone_<i>` text, or `None`                      |
| `report`   | last `/swarm/report/drone_<i>` text, or `None`                   |

All fields are null-safe: if the source topic hasn't been heard yet, the field
is `None` and the frontend renders `—`.

## Error / edge handling

- Missing topic → `None` → UI shows `—` (same pattern as today's position fields).
- Battery `remaining == -1` (unknown) → `None`, **not** `0`.
- Stale telemetry: keep last value (current behavior; `bridge.latest` returns the
  most recent message).
- `N` can change at runtime: frontend already rebuilds on `s.n !== built`; extend
  that rebuild to also (re)build thumbnails and reset the hero/selection if the
  selected id no longer exists (clamp to a valid id).

## Testing

Matches the repo's light testing style:

- Unit test the enum→string mapping (nav_state values incl. an unknown one →
  `#<n>`; arming_state → armed bool).
- Unit test the `/state` shape: with `bridge.latest` mocked to return canned
  VehicleLocalPosition / VehicleStatus / BatteryStatus, assert the per-drone dict
  has the new keys with correctly derived values (speed = hypot, heading in
  0–360, batt None when remaining = -1).
- Manual verification against a running sim before declaring done: select each
  drone, confirm hero swaps, metrics populate and update live, map/thumb clicks
  select, and badges color-code correctly.

## Out of scope (YAGNI)

- No historical charts / time-series.
- No per-drone control buttons (arm/takeoff) from the UI — commands still go
  through the chat/commander.
- No persistence of selection across reloads.
- No new external dependencies.
