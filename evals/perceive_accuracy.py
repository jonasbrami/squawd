"""perceive_accuracy — LIVE contact-position accuracy harness (the M2 gate
instrument). Runs INSIDE the sim container:

  blob "target" detections of mov_1 → footpoint → bearing+attitude → world →
  horizontal error vs GzPoses truth, measured through a 12 m/s transit.

Reports p50/p95 overall and by slant-range bucket (the gate: p50 < 5 m at
≤30 m for the GROUND mover, attitude path exercised by the fast transit).

Run:  docker exec pilot-sim bash -lc 'uv run --no-project python evals/perceive_accuracy.py'
      ... --backend onnx   # A/B vs the trained artifact (M2.5 gate)
"""
import argparse
import asyncio
import math
import os
import statistics
import sys
import time

sys.path.insert(0, "/workspace")

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.flight.ops import FlightOps
from agents.perception.projection import (contact_world, pixel_to_angles,
                                          ray_support_range)
from agents.vision.backends import ColorBlobBackend, OnnxBackend

TARGET = "mov_1"
TRANSIT_S = 90.0   # ~25s outbound + ≥60s aimed hover ≥ one full mover orbit
                   # (period ~63s): guarantees close passes in the <=30m bucket


def bucket(r):
    return "<=30m" if r <= 30.0 else "30-60m" if r <= 60.0 else ">60m"


def make_backend(name: str):
    if name == "blob":
        return ColorBlobBackend()
    if name == "onnx":
        return OnnxBackend("/workspace/models/mover-nano-seg-v1.onnx",
                           "/workspace/models/mover-nano-seg-v1.json")
    raise SystemExit(f"unknown backend {name!r} (blob|onnx)")


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="blob", choices=["blob", "onnx"])
    args = ap.parse_args()
    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    gz = GzPoses(os.environ.get("GZ_WORLD", "dynamic"), [TARGET])
    rec = Px4StateRecorder(bridge, world, i=0,
                           sim_time_ref=gz.sim_time)
                           # physics-rate pose stamps, ~ms latency — the 10 Hz
                           # camera stamp made attitude_at() fall off the
                           # buffer's newest edge on ~25% of frames
    backend = make_backend(args.backend)
    print(f"backend: {args.backend}", flush=True)
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    ops = FlightOps(system, world, bridge, 0, 1, gzposes=gz)
    bridge.start()
    rec.start()
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break

    print("waiting for frames + mover truth…", flush=True)
    truth0 = None
    for _ in range(150):
        truth0 = gz.poses().get(TARGET)
        if cameras.has(0) and truth0:
            break
        await asyncio.sleep(0.2)
    if not truth0:
        print("NO mover truth after 30s — GzPoses feed dead?", flush=True)
        return 1
    print(f"mov_1 starts at E{truth0[0]:.0f} N{truth0[1]:.0f}", flush=True)
    errors: list[tuple[float, float]] = []

    async def measure():
        last = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < TRANSIT_S:
            f = cameras.snapshot(0)
            if f is not None and f.seq != last:
                last = f.seq
                truth = gz.poses().get(TARGET)
                att = world.attitude_at(f.sim_stamp)
                st = world.drone_state(bridge, 0)
                # re-aim on EVERY new frame at the live bearing (10 Hz cadence):
                # a slow or lead-aimed chase parks the mover at the frame's
                # trailing edge where the blob clips and the footpoint row is
                # garbage (observed: 5x worse p50 vs frame-cadence aiming).
                # (airborne only — no goto spam while still on the ground)
                if truth and st and st[2] > 2.0:
                    dx, dy = truth[0] - st[0], truth[1] - st[1]
                    if math.hypot(dx, dy) > 5.0:
                        yaw = math.degrees(math.atan2(dx, dy)) % 360
                        try:
                            await ops.goto(east=hover_e, north=hover_n, up=6.0,
                                           heading=f"{yaw:.1f}", wait=False)
                        except Exception:
                            pass
                if truth and att and st:
                    # association gate on bearing ±10° AND elevation ±8°
                    # (fable-R3 hygiene): azimuth alone let the AERIAL orange
                    # movers (mov_0/3/4 at z=10-12) slip in for the model —
                    # the blob self-gates on area, so the contamination was
                    # asymmetric. Elevation computed from truth vs drone alt.
                    true_ax = (math.atan2(truth[0] - st[0], truth[1] - st[1])
                               - st[3] + math.pi) % (2 * math.pi) - math.pi
                    hd = math.hypot(truth[0] - st[0], truth[1] - st[1])
                    true_dep = math.atan2(st[2] - truth[2], hd)  # truth z vs alt
                    for d in backend.infer(f, 0.2):
                        if d.cls != "target":
                            continue
                        ax, ay = pixel_to_angles(d.footpoint[0], d.footpoint[1],
                                                 f.width, f.height)
                        dax = (ax - true_ax + math.pi) % (2 * math.pi) - math.pi
                        if abs(dax) > math.radians(10.0):
                            continue
                        # elevation: pixel depression vs true depression
                        if abs((ay - att[1]) - true_dep) > math.radians(8.0):
                            continue
                        rng = ray_support_range(ax, ay, roll=att[0],
                                                pitch=att[1], alt=st[2],
                                                support_z=0.6)
                                                # mov_1's box (1.2m tall) is
                                                # centered at z=1.2 -> its base
                                                # (the blob's footpoint) rests
                                                # at z=0.6, not z=0 — measured
                                                # live (blob floats over its
                                                # shadow); z=0 overshoots range
                                                # ~15% (root-caused 2026-07-21)
                        if rng is None:
                            continue
                        e, n = contact_world(st[0], st[1], st[3], ax, rng)
                        err = math.hypot(e - truth[0], n - truth[1])
                        # bucket by TRUTH slant, not the estimated rng (fable-R3:
                        # an underestimated 40 m sample leaked multi-meter error
                        # into the <=30 m gate bucket)
                        slant_true = math.sqrt(hd * hd + (st[2] - truth[2]) ** 2)
                        errors.append((slant_true, err))
            await asyncio.sleep(0.03)

    print("takeoff, then hovering on the mover's circle…", flush=True)
    print(await ops.take_off(6.0), flush=True)
    m = asyncio.create_task(measure())   # measures THROUGH the 12 m/s outbound
                                         # transit and the aimed hover after it
    # hover near the circle's current north edge: the rover passes through the
    # frame repeatedly while we measure (circle center (70,-100), r=35).
    # 6 m, not 12: the IMX214's half-vFOV is ~21°, so from 12 m the mover
    # leaves the frame below ~28 m horizontal and the <=30 m slant bucket is
    # geometrically empty (observed: 0 samples); from 6 m the whole 13–30 m
    # slant band stays in view.
    hover_e, hover_n = truth0[0] - 20.0, truth0[1]
    print(await ops.goto(east=hover_e, north=hover_n, up=6.0), flush=True)
    await m
    print(await ops.land(), flush=True)

    if not errors:
        print("NO MEASUREMENTS — mover never ranged in frame", flush=True)
        return 1
    overall = sorted(e for _, e in errors)
    p50 = statistics.median(overall)
    p95 = overall[int(0.95 * (len(overall) - 1))] if len(overall) >= 2 else None
    print(f"\nn={len(errors)}  p50={p50:.2f}m  p95={p95:.2f}m")
    by: dict[str, list] = {}
    for r, e in errors:
        by.setdefault(bucket(r), []).append(e)
    for b, es in sorted(by.items()):
        es = sorted(es)
        print(f"  {b}: n={len(es)} p50={statistics.median(es):.2f}m "
              f"p95={es[int(0.95 * (len(es) - 1))]:.2f}m")
    gate = by.get("<=30m") and statistics.median(by["<=30m"]) < 5.0
    print(f"\nM2 GATE (p50 < 5m at <=30m): {'PASS' if gate else 'FAIL'}")
    return 0 if gate else 1


if __name__ == "__main__":
    code = asyncio.run(main())
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)   # skip gz-transport Node destructors at interpreter
                     # teardown ("terminate called without an active exception",
                     # exit 134) — the run's result is already printed
