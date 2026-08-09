"""M4 recorded-set capture driver (IN-CONTAINER, one process, whole matrix).

Stages the drone through parked-hover spots (FlightOps.goto — the
w3_reposition.py pattern) and saves raw lossless gz frames + JSON sidecars
for the deep-perception M4 metrics set. Captures are yaw-only/parked: goto
transits between spots, settle, then snapshot. Mover shots wait for the
target mover (live GzPoses truth) to enter a range band inside the camera
cone before snapping — the sidecar records the TRUE range/bearing/aspect.

Each sidecar: {tag, seq, sim_stamp, w, h, pose:{e,n,alt,heading_deg},
att:{roll,pitch,yaw}_deg, movers:{name:[x,y,z]}, shot:{...},
mover_window: {...}|None, missed_window: bool}.

Run INSIDE pilot-sim:
  cd /workspace && PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project \
      python evals/out/deep_m4/capture_set.py [--only tag1,tag2] [--no-land]
"""
import argparse
import asyncio
import json
import math
import os
import time

from PIL import Image

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.telemetry import _quat_to_rpy
from agents.flight.ops import FlightOps
from agents.world import World
from mavsdk import System

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
MOVERS = ["car_1", "car_2", "car_3", "walker_1", "walker_2"]

# (tag, e, n, alt, yaw_deg, mover_window or None)
# mover_window = (mover_name, rmin, rmax, cone_deg) — snap when the mover's
# true range from the drone is in [rmin, rmax] AND its bearing is within
# cone_deg of the shot yaw.
SHOTS = [
    # A — house cluster, parked S of house_1 (alt 9: house base in frame)
    ("a1_house1_close30", 49, -18, 9, 185, None),
    ("a2_house2_oak_far", 49, -18, 9, 218, None),
    ("a3_empty_se",       49, -18, 9, 135, None),
    # B — car_1 close, parked N of the south leg (50,-18; the -42 spot is
    # inside house_1's footprint — FlightOps guard). alt 4.5: 11+ m visible.
    # Bands/yaws derived per-leg so the target sits inside the hfov cone.
    ("b1_car_12_side",    50, -18, 4.5, 180, ("car_1", 11, 14.5, 32)),
    ("b2_car_22_rearq",   50, -18, 4.5, 115, ("car_1", 20, 24, 32)),
    ("b2b_car_27_rearq",  50, -18, 4.5, 55,  ("car_1", 24, 30, 32)),
    ("b3_car_40_rearq",   50, -18, 4.5, 30,  ("car_1", 37, 43, 32)),
    ("b4_car_21_frontq",  50, -18, 4.5, 292, ("car_1", 19, 24, 32)),
    # C — gas station overlook (alt 8: near base in frame)
    ("c1_gasstation_35",  64, 26, 8, 45,  None),
    ("c2_oak_46",         64, 26, 8, 108, None),
    ("c3_car_30_front",   64, 26, 8, 275, ("car_1", 28, 36, 32)),
    ("c4_houses_far",     64, 26, 8, 205, None),
    # D — truck (car_3) loop, parked inside the loop at low alt
    ("d1_truck_15_side",  102, -14, 4.5, 0,   ("car_3", 13, 17, 32)),
    ("d2_truck_18_side",  102, -14, 4.5, 90,  ("car_3", 15, 19, 35)),
    ("d2b_truck_22_side", 102, -14, 4.5, 60,  ("car_3", 20, 24, 35)),
    ("d3_truck_29_frontq", 102, -14, 4.5, 210, ("car_3", 27, 32, 32)),
    ("d4_house1_far63",   102, -14, 4.5, 240, None),
    # E — walker_1 sidewalk, low alt (50,-24: closest path point 12 m in frame)
    ("e1_walker_13_side", 50, -24, 3.5, 180, ("walker_1", 12, 15, 32)),
    ("e2_walker_17_side", 50, -24, 3.5, 140, ("walker_1", 15, 19, 32)),
    ("e3_walker_22_side", 50, -24, 3.5, 124, ("walker_1", 20, 24, 32)),
    ("e4_pine_43",        50, -24, 3.5, 323, None),
    ("e5_pole_25",        50, -24, 3.5, 247, None),
    # F — high overview from home: far buildings + FP-bait empties
    ("f1_gasstation_100", 0, 0, 25, 60,  None),
    ("f2_house1_70",      0, 0, 25, 135, None),
    ("f3_empty_w",        0, 0, 25, 270, None),
    ("f4_pine2_pole2_far", 0, 0, 25, 315, None),
    ("f5_empty_sw",       0, 0, 25, 225, None),
    ("f6_empty_e",        0, 0, 25, 90,  None),
    ("f7_empty_n",        0, 0, 25, 0,   None),
]

CAM_CONE_DEG = 34.5          # hfov/2 — object must be inside this to be visible


class Attitude:
    """Latest vehicle_attitude as (roll, pitch, yaw) radians."""

    def __init__(self, bridge):
        from px4_msgs.msg import VehicleAttitude
        self.rpy = (0.0, 0.0, 0.0)
        bridge.subscribe("/px4_0/fmu/out/vehicle_attitude", VehicleAttitude,
                         callback=self._cb)

    def _cb(self, m):
        q = getattr(m, "q", None)
        if q is None:
            return
        self.rpy = _quat_to_rpy(float(q[0]), float(q[1]), float(q[2]),
                                float(q[3]))


def _ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


async def snap(cameras, world, bridge, gz, att, tag, shot, window_hit):
    # yaw-settle gate: goto's arrival wait is position-only; big slews are
    # still turning when it returns (f-series 45 deg misses). Wait until the
    # heading is within 8 deg of the shot yaw (10 s cap).
    want = shot[4] % 360.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        st = world.drone_state(bridge, 0)
        if st and abs(_ang_diff(math.degrees(st[3]) % 360.0, want)) <= 8.0:
            break
        await asyncio.sleep(0.2)
    f = None
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        f = cameras.snapshot(0)
        if f is not None:
            break
        await asyncio.sleep(0.2)
    if f is None:
        print(f"[{tag}] NO FRAME — skipped", flush=True)
        return False
    st = world.drone_state(bridge, 0)
    e, n, alt, heading = st if st else (None, None, None, None)
    roll, pitch, _yaw = att.rpy
    movers = gz.poses()
    Image.frombytes("RGB", (f.width, f.height), f.rgb).save(
        os.path.join(OUT_DIR, f"{tag}.png"))
    meta = {
        "tag": tag, "seq": f.seq, "sim_stamp": f.sim_stamp,
        "w": f.width, "h": f.height,
        "pose": {"e": e, "n": n, "alt": alt,
                 "heading_deg": math.degrees(heading) if heading is not None
                 else None},
        "att_deg": {"roll": math.degrees(roll), "pitch": math.degrees(pitch),
                    "yaw": math.degrees(_yaw)},
        "movers": {k: list(v) for k, v in movers.items()},
        "shot": {"e": shot[1], "n": shot[2], "alt": shot[3], "yaw": shot[4]},
        "mover_window": window_hit,
        "missed_window": window_hit == "timeout",
    }
    with open(os.path.join(OUT_DIR, f"{tag}.json"), "w") as fh:
        json.dump(meta, fh, indent=1)
    print(f"[{tag}] seq={f.seq} stamp={f.sim_stamp:.1f} "
          f"pose=({e:.1f},{n:.1f},{alt:.1f}) hdg={math.degrees(heading):.0f} "
          f"win={window_hit}", flush=True)
    return True


async def wait_mover(gz, world, bridge, name, rmin, rmax, cone, yaw,
                     timeout=100.0):
    """Poll live mover truth until (range in band) and (bearing within cone of
    yaw). Returns a descriptor dict, or 'timeout'."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = world.drone_state(bridge, 0)
        poses = gz.poses()
        if st and name in poses:
            e, n, _alt, _h = st
            mx, my, _mz = poses[name]
            de, dn = mx - e, my - n
            rng = math.hypot(de, dn)
            bearing = math.degrees(math.atan2(de, dn)) % 360.0
            if rmin <= rng <= rmax and abs(_ang_diff(bearing, yaw)) <= cone:
                return {"mover": name, "range_m": round(rng, 1),
                        "bearing_deg": round(bearing, 1),
                        "at": [round(mx, 1), round(my, 1)]}
        await asyncio.sleep(0.2)
    return "timeout"


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None, help="comma-separated tag filter")
    ap.add_argument("--no-land", action="store_true")
    ap.add_argument("--takeoff-alt", type=float, default=9.0)
    args = ap.parse_args()
    only = set(args.only.split(",")) if args.only else None
    os.makedirs(OUT_DIR, exist_ok=True)

    bridge = RosBridge(node_name="m4_capture")
    from px4_msgs.msg import VehicleLocalPosition
    bridge.subscribe("/px4_0/fmu/out/vehicle_local_position",
                     VehicleLocalPosition)
    world = World()
    gz = GzPoses(os.environ.get("GZ_WORLD", "demo"), MOVERS)
    att = Attitude(bridge)
    cameras = GzCameras(1)
    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    ops = FlightOps(drone, world, bridge, 0, 1, contacts=gz)
    bridge.start()
    await asyncio.sleep(2.0)

    async for air in drone.telemetry.in_air():
        in_air = air
        break
    if not in_air:
        print(f"taking off to {args.takeoff_alt:.0f} m…", flush=True)
        await drone.action.set_takeoff_altitude(args.takeoff_alt)
        await drone.action.takeoff()
        for _ in range(400):
            await asyncio.sleep(0.2)
            pos = await anext(drone.telemetry.position())
            if abs(pos.relative_altitude_m - args.takeoff_alt) < 0.8:
                break
        print(f"airborne at {pos.relative_altitude_m:.1f} m", flush=True)
        if pos.relative_altitude_m < args.takeoff_alt - 2.0:
            raise SystemExit(
                f"TAKEOFF FAILED (alt {pos.relative_altitude_m:.1f} after "
                f"settle wait) — aborting BEFORE the matrix (preflight?)")

    done = skip = 0
    for shot in SHOTS:
        tag, e, n, alt, yaw, win = shot
        if only and tag not in only:
            continue
        print(f"--- {tag}: goto E{e} N{n} alt {alt} yaw {yaw}", flush=True)
        try:
            print(f"    {await ops.goto(east=e, north=n, up=alt, heading=str(yaw))}",
                    flush=True)
        except Exception as exc:
            print(f"    GOTO FAILED: {exc} — skipping shot", flush=True)
            skip += 1
            continue
        await asyncio.sleep(1.5)                    # settle
        hit = None
        if win:
            name, rmin, rmax, cone = win
            hit = await wait_mover(gz, world, bridge, name, rmin, rmax,
                                   cone, yaw)
            if hit == "timeout":
                print(f"    mover window TIMEOUT — snapping anyway",
                      flush=True)
        if await snap(cameras, world, bridge, gz, att, tag, shot, hit):
            done += 1
        else:
            skip += 1

    print(f"captured {done}, skipped {skip}", flush=True)
    if not args.no_land:
        print(await ops.land(), flush=True)
        await asyncio.sleep(6.0)


if __name__ == "__main__":
    asyncio.run(main())
