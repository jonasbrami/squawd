"""perceive_report — LIVE driver for the M5 accuracy report (runs inside the
sim container, on the PERCEIVE world): hover at a level vantage over the
plaza, record frames + timestamp-joined projected truth boxes for every mover
(mov_true + the distinct decoys), then run the backend + accuracy_report
OFFLINE (fixtures discipline — no live threaded detector in the grading path)
and dump the JSON into evals/out/.

Truth projection is the level-attitude inverse (perceive_eval.project_truth_box):
frames with |roll| or |pitch| > 2 deg are dropped (the inverse would skew).
Decoy classes are EXPECTED recall-0 under the single-class blob — their rows
quantify decoy rejection; the target class's FP count catches color bleed.
ID-switch/fragmentation populate only with a track-id backend (ultralytics
track mode); blob/onnx report None (fixtures in tests/evals/test_perceive_eval.py
prove the metric math).

Run:  docker exec pilot-sim bash -lc 'uv run --no-project python \
        evals/perceive_report.py [--backend blob|onnx] [--record-s 60]'
"""
import argparse
import asyncio
import json
import math
import os
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
from evals.perceive_eval import accuracy_report, project_truth_box

VANTAGE = (40.0, -70.0, 3.0)   # plaza rim; alt 3 keeps the rover's near arc
                               # inside the 21° half-vFOV (alt 6 pushes the
                               # whole visible band past ~56 m — blob starves)
VANTAGE_HEADING = "150"        # explicit, approach-independent: a travel
                               # heading faces the APPROACH leg, and if the
                               # drone didn't come from home that can put the
                               # plaza outside the FOV for the whole run
                               # (observed live 2026-07-22: 586 frames, zero
                               # truth boxes, zero dets)
LEVEL_GATE_RAD = math.radians(2.0)


def make_backend(name: str):
    from agents.vision.backends import ColorBlobBackend, OnnxBackend
    if name == "blob":
        return ColorBlobBackend()
    if name == "onnx":
        return OnnxBackend("/workspace/models/mover-nano-seg-v1.onnx",
                           "/workspace/models/mover-nano-seg-v1.json")
    raise SystemExit(f"unknown backend {name!r} (blob|onnx)")


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="blob", choices=["blob", "onnx"])
    ap.add_argument("--record-s", type=float, default=60.0)
    args = ap.parse_args()

    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    movers = {m["name"]: m for m in world.movers}
    if not movers:
        print("no movers in this world — launch with PX4_GZ_WORLD=perceive",
              flush=True)
        return 1
    gz = GzPoses(os.environ.get("GZ_WORLD", "perceive"), sorted(movers))
    rec = Px4StateRecorder(bridge, world, i=0, sim_time_ref=gz.sim_time)
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    ops = FlightOps(system, world, bridge, 0, 1)
    bridge.start()
    rec.start()
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    for _ in range(150):
        if cameras.has(0) and gz.poses():
            break
        await asyncio.sleep(0.2)

    print(f"vantage hover {VANTAGE} heading {VANTAGE_HEADING} — recording "
          f"{args.record_s:.0f}s…", flush=True)
    print(await ops.take_off(VANTAGE[2]), flush=True)
    print(await ops.goto(east=VANTAGE[0], north=VANTAGE[1], up=VANTAGE[2],
                         heading=VANTAGE_HEADING), flush=True)

    frames, truths = [], []
    dropped_att = 0
    last = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.record_s:
        f = cameras.snapshot(0)
        if f is not None and f.seq != last:
            last = f.seq
            att = world.attitude_at(f.sim_stamp)
            st = world.drone_state(bridge, 0)
            poses = gz.poses()
            if att is None or st is None:
                continue
            if abs(att[0]) > LEVEL_GATE_RAD or abs(att[1]) > LEVEL_GATE_RAD:
                dropped_att += 1
                continue
            row = {"stamp": f.sim_stamp, "boxes": {}, "ids": {}, "ranges": {}}
            for name, m in movers.items():
                pos = poses.get(name)
                if pos is None:
                    continue
                s = m["shape"]
                box, slant = project_truth_box(
                    st[0], st[1], st[2], st[3], pos[0], pos[1], m["z"],
                    max(s["w"], s["d"]), s["h"], f.width, f.height)
                if box is None:
                    continue
                cls = "target" if name == "mov_true" else m["kind"]
                row["boxes"].setdefault(cls, []).append(box)
                row["ids"].setdefault(cls, []).append(name)
                row["ranges"].setdefault(cls, []).append(slant)
            frames.append(f)
            truths.append(row)
        await asyncio.sleep(0.02)
    print(await ops.land(), flush=True)
    print(f"recorded {len(frames)} joined frames ({dropped_att} dropped for "
          "attitude)", flush=True)
    if len(frames) < 10:
        print("TOO FEW FRAMES — camera or truth feed dead?", flush=True)
        return 1

    backend = make_backend(args.backend)
    # conf 0.25 — the production Detector's value (pilot/run.py): the blob's
    # far-range scores (0.35-0.45) would starve at the 0.45 library default,
    # and the report must measure the SAME operating point the pilot flies.
    rep = accuracy_report(frames, truths, backend, conf=0.25)
    rep["backend"] = args.backend
    rep["note"] = ("decoy classes are expected recall-0 under single-class "
                   "backends; id metrics need a track-id backend")
    out_dir = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir,
                        f"perceive_accuracy_{time.strftime('%Y%m%d-%H%M%S')}.json")
    with open(path, "w") as fh:
        json.dump(rep, fh, indent=2)
    print(json.dumps(rep, indent=2), flush=True)
    tgt = rep["per_class"].get("target")
    if tgt:
        p50 = rep["center_err_p50"]
        print(f"\ntarget: precision={tgt['precision']:.2f} "
              f"recall={tgt['recall']:.2f} (tp={tgt['tp']} fp={tgt['fp']} "
              f"fn={tgt['fn']})  center p50="
              f"{'%.1f' % p50 if p50 is not None else 'n/a'}px")
    else:
        print("\nNO target-class matches at all — detector saw nothing",
              flush=True)
    print(f"report -> {path}", flush=True)
    return 0


if __name__ == "__main__":
    code = asyncio.run(main())
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)   # skip gz-transport Node destructors at interpreter teardown
