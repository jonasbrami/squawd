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


async def soft_reset(systems, world, bridge, n, tol_m=5.0, timeout_s=120.0,
                     poll_interval_s=1.0) -> ResetResult:
    # timeout_s covers the WORST case now that check_home also gates altitude:
    # a deadline-cut cell can leave the drone ~150m out at 20m up — RTL transit
    # (~30s) + descent (~20-40s at land speed) blew the old 60s window and tripped
    # the infra fuse on healthy sims.
    results = await asyncio.gather(
        *[s.action.return_to_launch() for s in systems],
        return_exceptions=True)
    errors = [f"drone_{i}: {r}" for i, r in enumerate(results)
              if isinstance(r, Exception)]
    if errors:
        return ResetResult(False, "RTL command failed: " + "; ".join(errors))
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        r = check_home(world, bridge, n, tol_m)
        if r.ok:
            return r
        await asyncio.sleep(poll_interval_s)
    return check_home(world, bridge, n, tol_m)
