"""Soft reset between runs: RTL all drones home, then verify the world is clean.

RTL (return-to-launch) brings each drone back to its fixed spawn XY and lands WITHOUT
teleporting the vehicle, so the EKF stays converged (the failure mode that makes naive
soft-resets leaky). check_home is the health gate: if any drone isn't near home, the
caller escalates to a full sim teardown. Pure geometry here is unit-tested; the live
RTL loop is bounded by timeout_s."""
import asyncio
import math
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class ResetResult:
    ok: bool
    reason: str


def home_xy(world, i: int) -> tuple[float, float]:
    return (world.spawn_x, world.spawn_spacing * i)


def check_home(world, bridge, n: int, tol_m: float, alt_tol_m: float = 2.5) -> ResetResult:
    for i in range(n):
        xy = world.world_xy(bridge, i)
        if xy is None:
            return ResetResult(False, f"drone_{i} has no fix")
        hx, hy = home_xy(world, i)
        d = math.hypot(xy[0] - hx, xy[1] - hy)
        if d > tol_m:
            return ResetResult(False, f"drone_{i} {d:.1f}m from home (tol {tol_m:g})")
        # 2D-only checking was leaky: a drone hovering 12m over home passed the gate
        # and the next cell's take_off started from altitude instead of the ground.
        if len(xy) > 2 and xy[2] > alt_tol_m:
            return ResetResult(False, f"drone_{i} still airborne at {xy[2]:.1f}m")
    return ResetResult(True, "all drones home")


FERRY_ALT_M = 40.0   # above every building in current worlds — the ferry's
                     # straight-line hop home must not itself hit obstacles


async def _ferry_home(s, world, bridge, i: int, hx: float, hy: float,
                      poll_interval_s: float) -> str:
    """Fly a disarmed, landed-away drone back to WORLD home: arm+takeoff (retried),
    goto_location at home, land. Flies at FERRY_ALT_M (a 10m ferry collided with
    the obstacles world's buildings on its way home). Returns '' on success, else
    the last error note. Never raises — check_home stays the arbiter."""
    from agents.core.geo import GeoPoint, offset_point

    # arm + takeoff, retried through the transient post-land COMMAND_DENIED;
    # hold() first: PX4's Land nav_state has mode_req_prevent_arming, so arm()
    # stays denied after any land() until the intention leaves Land.
    airborne = False
    err = ""
    for attempt in range(4):
        try:
            await s.action.hold()
            await s.action.set_takeoff_altitude(FERRY_ALT_M)
            await s.action.arm()
            await s.action.takeoff()
        except Exception as e:
            err = f"drone_{i} ferry arm/takeoff attempt {attempt}: {e}"
            await asyncio.sleep(2.0)
            continue
        for _ in range(int(20 / max(poll_interval_s, 0.05))):
            await asyncio.sleep(poll_interval_s)
            xy = world.world_xy(bridge, i)
            if xy is not None and len(xy) > 2 and xy[2] > 3.0:
                airborne = True
                break
        if airborne:
            break
        err = f"drone_{i} ferry attempt {attempt}: takeoff never left ground"
    if not airborne:
        return err

    # goto WORLD home (NOT RTL — arming just moved PX4's home to the ferry spot)
    try:
        me = world.world_xy(bridge, i)
        async for pos in s.telemetry.position():
            origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
            break
        tgt = offset_point(origin, hy - me[1], hx - me[0], FERRY_ALT_M - me[2])
        await s.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                     tgt.absolute_altitude_m, 0.0)
    except Exception as e:
        return f"drone_{i} ferry goto failed: {e}"
    for _ in range(int(90 / max(poll_interval_s, 0.05))):
        await asyncio.sleep(poll_interval_s)
        xy = world.world_xy(bridge, i)
        if xy is not None and math.hypot(xy[0] - hx, xy[1] - hy) <= 3.0:
            break
    else:
        return f"drone_{i} ferry never reached home"
    try:
        await s.action.land()
    except Exception as e:
        return f"drone_{i} ferry land failed: {e}"
    # Wait for TOUCHDOWN (disarm), not just the land command. The ferry re-armed
    # away from home, so this drone's PX4 home is the STRANDING POINT — if
    # soft_reset's RTL wave catches it still airborne over world home, RTL
    # faithfully flies it straight back to where it was stranded (observed live:
    # ferry 'succeeded', drone teleported back to the w2 checkpoint).
    for _ in range(int(60 / max(poll_interval_s, 0.05))):
        await asyncio.sleep(poll_interval_s)
        try:
            async for a in s.telemetry.armed():
                if a is False:
                    return ""
                break
        except Exception:
            pass
    return f"drone_{i} ferry landed-wait timed out (still armed at home)"


async def soft_reset(systems, world, bridge, n, tol_m=5.0, timeout_s=120.0,
                     poll_interval_s=1.0) -> ResetResult:
    # timeout_s covers the WORST case now that check_home also gates altitude:
    # a deadline-cut cell can leave the drone ~150m out at 20m up — RTL transit
    # (~30s) + descent (~20-40s at land speed) blew the old 60s window and tripped
    # the infra fuse on healthy sims.

    # A cell can legitimately END with the drone LANDED away from home (agents land
    # after tasks). RTL on a disarmed vehicle is a no-op — PX4 enters RETURN_TO_LAUNCH
    # mode and just sits there (observed live). And RTL after RE-ARMING away from
    # home is a TRAP: PX4 records its home position AT ARMING, so the ferried drone
    # "returns" to the exact spot it was re-armed at (observed live: 99.9m from home
    # after every ferry). So the ferry must fly to WORLD home itself: arm + takeoff
    # (retried through PX4's transient COMMAND_DENIED), goto world home, land there —
    # each phase bounded. The trigger is the DISARMED state, not an altitude guess
    # (a parked drone's EKF altitude drifts ~2m, past any grounded threshold).
    # set_speed persists via MPC_XY_CRUISE — restore the default so one cell's
    # speed choice can't leak into the next (a pass at a speed the agent never
    # commanded would be a phantom result).
    for s in systems:
        try:
            await s.param.set_param_float("MPC_XY_CRUISE", 5.0)
        except Exception:
            pass
    ferry_err = ""
    ferried: set[int] = set()
    for i, s in enumerate(systems):
        xy = world.world_xy(bridge, i)
        if xy is None or len(xy) < 3:
            continue
        hx, hy = home_xy(world, i)
        if math.hypot(xy[0] - hx, xy[1] - hy) <= tol_m:
            continue
        armed = None
        try:
            async for a in s.telemetry.armed():
                armed = a
                break
        except Exception:
            pass
        if armed is False or (armed is None and xy[2] < 2.5):
            ferried.add(i)
            ferry_err = await _ferry_home(s, world, bridge, i, hx, hy, poll_interval_s)

    # NEVER RTL a ferried drone: it re-armed away from home, so its PX4 home is
    # the stranding point — RTL would undo the ferry (see _ferry_home's landing
    # wait for the same trap when the RTL wave races the ferry's descent).
    rtl_ids = [i for i in range(len(systems)) if i not in ferried]
    results = await asyncio.gather(
        *[systems[i].action.return_to_launch() for i in rtl_ids],
        return_exceptions=True)
    errors = [f"drone_{i}: {r}" for i, r in zip(rtl_ids, results)
              if isinstance(r, Exception)]
    if errors:
        return ResetResult(False, "RTL command failed: " + "; ".join(errors))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        r = check_home(world, bridge, n, tol_m)
        if r.ok:
            return r
        await asyncio.sleep(poll_interval_s)
    r = check_home(world, bridge, n, tol_m)
    if not r.ok and ferry_err:
        return ResetResult(False, f"{r.reason} ({ferry_err})")
    return r
