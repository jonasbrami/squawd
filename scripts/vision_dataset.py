#!/usr/bin/env python3
"""vision_dataset — sim render capture + geometric auto-labels (M2.5).

Runs INSIDE the sim container. Flies a capture pattern over the dynamic-world
movers, saves camera frames, and writes YOLO-seg labels computed from GROUND
TRUTH (dynamic_pose/info): each mover's 3D box corners are projected through
the same pinhole intrinsics the projection path uses (agents/perception/
projection.py: hfov 1.204 rad, 640x360), the 2D hull becomes the seg polygon.

Classes from the sidecar `kind`: target=0 (orange movers), obstacle=1
(blue-grey). Every Nth frame goes to val/. A few debug images with the drawn
bbox ship alongside for eyeball verification.

  docker exec pilot-sim bash -lc 'uv run --no-project python \
    scripts/vision_dataset.py --out /workspace/evals/out/dataset_v1 --per-class 600'

NOTE (scope): the design names this tools/vision_dataset.py; the goal scope
limits changes to scripts/ among code dirs, so it lives here.
"""
import argparse
import asyncio
import io
import json
import math
import os
import sys
import time

sys.path.insert(0, "/workspace")

# Third-party/sim imports (mavsdk, gz, PIL, agents.core.camera) are LAZY —
# they live inside main()/Truth.__init__ so this module stays importable on
# the host (no sim deps) for scripts/demo_dataset.py, which reuses Truth,
# hull and the labeling geometry (W2.5b demo-coco path).

FX = 320.0 / math.tan(1.204 / 2.0)          # IMX214 intrinsics (projection.py)
FY = FX                                      # square pixels, 640x360
CX, CY = 320.0, 180.0
CAM_OFFSET = (0.132, 0.0, 0.261)             # camera in body frame (approx)
DRONE = "x500_depth_0"


# ---------- truth ----------

class Truth:
    """Latest (pos, quat) per model from dynamic_pose/info."""

    def __init__(self, world: str):
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        self._world = world
        self._models: dict[str, tuple] = {}
        self._node = GzNode()
        self._node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info",
                             self._on)

    def _on(self, msg):
        for p in msg.pose:
            self._models[p.name] = (
                p.position.x, p.position.y, p.position.z,
                p.orientation.w, p.orientation.x, p.orientation.y,
                p.orientation.z)

    def get(self, name):
        return self._models.get(name)


def yaw_pitch(qw, qx, qy, qz):
    """ENU quaternion -> (bearing_from_north_cw, pitch_up) matching the
    projection path's conventions (heading: 0=N, +cw; pitch: nose-up +)."""
    # ENU yaw from +X (east), CCW+
    yaw_e = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    sp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sp)                     # ENU: nose-up positive
    bearing = (math.pi / 2.0 - yaw_e + math.pi) % (2 * math.pi) - math.pi
    return bearing, pitch


# ---------- labeling ----------

def project_point(pe, pn, pz, cam, bearing, pitch):
    """World point -> (u, v) or None when behind the camera."""
    ce, cn, cu = cam
    de, dn, dz = pe - ce, pn - cn, pz - cu
    hd = math.hypot(de, dn)
    if hd < 1e-6:
        return None
    ax = (math.atan2(de, dn) - bearing + math.pi) % (2 * math.pi) - math.pi
    # ray elevation vs boresight (pitch up +): depression-positive ay
    el = math.atan2(dz, hd)
    ay = -(el) - pitch
    # point must be in front: |ax| < ~90deg
    if abs(ax) > math.radians(89.0):
        return None
    u = CX + FX * math.tan(ax)
    v = CY + FY * math.tan(ay)
    return (u, v)


def hull(points):
    """Monotone-chain convex hull of [(x,y), ...] (no collinear repeats)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def label_mover(mov, cam, bearing, pitch, w, h):
    """-> (bbox, polygon) in pixels, or None when not visibly in frame."""
    cx_, cy_, cz_ = mov["_pos"]
    sw, sd, sh = mov["shape"]["w"], mov["shape"]["d"], mov["shape"]["h"]
    corners = []
    for dx in (-sw / 2, sw / 2):
        for dy in (-sd / 2, sd / 2):
            for dz in (-sh / 2, sh / 2):
                corners.append((cx_ + dx, cy_ + dy, cz_ + dz))
    pts = []
    for c in corners:
        pr = project_point(c[0], c[1], c[2], cam, bearing, pitch)
        if pr is not None:
            pts.append(pr)
    if len(pts) < 3:
        return None
    poly = hull(pts)
    poly = [(min(max(x, 0.0), w), min(max(y, 0.0), h)) for x, y in poly]
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if max(xs) - min(xs) < 6 or max(ys) - min(ys) < 6:
        return None                       # sub-pixel-ish: useless as a label
    return (min(xs), min(ys), max(xs), max(ys)), poly


# ---------- capture ----------

async def arm_takeoff_goto(s, alt, tgt_geo=None):
    from mavsdk.action import ActionError
    for _ in range(3):
        try:
            await s.action.hold()
            await s.action.arm()
            break
        except ActionError:
            await asyncio.sleep(1.5)
    else:
        raise RuntimeError("arm denied x3")
    await s.action.set_takeoff_altitude(alt)
    await s.action.takeoff()
    await asyncio.sleep(6)


async def main() -> int:
    from mavsdk import System
    from mavsdk.action import ActionError
    from PIL import Image, ImageDraw

    from agents.core.camera import GzCameras

    ap = argparse.ArgumentParser()
    ap.add_argument("--sidecar", default="/workspace/PX4-Autopilot/Tools/"
                    "simulation/gz/worlds/dynamic_boxes.json")
    ap.add_argument("--out", default="/workspace/evals/out/dataset_v1")
    ap.add_argument("--per-class", type=int, default=600)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--world", default=os.environ.get("GZ_WORLD", "dynamic"))
    args = ap.parse_args()

    cfg = json.load(open(args.sidecar))
    kinds = {"target": 0, "obstacle": 1}
    movers = [m for m in cfg["movers"] if m.get("kind") in kinds]
    for d in ("images/train", "images/val", "labels/train", "labels/val",
              "debug"):
        os.makedirs(os.path.join(args.out, d), exist_ok=True)

    truth = Truth(args.world)
    cams = GzCameras(1)
    s = System(mavsdk_server_address="127.0.0.1", port=50051)
    # connect BEFORE the noisy threads (gRPC starvation lesson, M2 report)
    await s.connect()
    async for st in s.core.connection_state():
        if st.is_connected:
            break
    for _ in range(150):
        if cams.has(0) and truth.get(DRONE):
            break
        await asyncio.sleep(0.2)
    print("frames+truth ok", flush=True)

    async def goto_enu(e, n, u, yaw=None):
        """World-ENU -> GPS via the LIVE fix, not a boot-time home: PX4's global
        position wanders while the EKF settles, and a home captured then sent
        the drone 350 m past the geofence (observed 2026-07-21)."""
        from agents.core.geo import GeoPoint, offset_point
        me = truth.get(DRONE)
        pos = await anext(s.telemetry.position())
        if me is not None:
            dn, de = n - me[1], e - me[0]
        else:
            dn, de = n, e
        if math.hypot(e, n) > 250.0:
            raise ValueError(f"capture target E{e:.0f} N{n:.0f} beyond 250 m — "
                             "capture grid is <=150 m by design")
        if not (1.0 <= u <= 30.0):
            raise ValueError(f"capture alt {u} outside 1–30 m grid")
        # offset_point ADDS up to the origin AMSL — ground-reference it or each
        # aim call ratchets the ceiling (observed: +5 m per aim to z=1051 m,
        # geofence failsafe). ground AMSL = current AMSL - current world z.
        ground_ams = pos.absolute_altitude_m - (me[2] if me is not None else 0.0)
        g = offset_point(GeoPoint(pos.latitude_deg, pos.longitude_deg,
                                  ground_ams), dn, de, u)
        if yaw is None and me is not None:
            yaw, _ = yaw_pitch(me[3], me[4], me[5], me[6])
            yaw = math.degrees(yaw) % 360
        await s.action.goto_location(g.latitude_deg, g.longitude_deg,
                                     g.absolute_altitude_m, yaw)

    counts = {0: 0, 1: 0}
    saved = 0

    def save_frame(f, labels):
        nonlocal saved
        split = "val" if saved % 10 == 9 else "train"
        stem = f"cap_{saved:05d}"
        img = Image.frombytes("RGB", (f.width, f.height), f.rgb)
        img.save(os.path.join(args.out, "images", split, stem + ".png"))
        with open(os.path.join(args.out, "labels", split, stem + ".txt"),
                  "w") as fh:
            for cls_id, poly in labels:
                norm = [f"{x / f.width:.5f}" if i % 2 == 0 else
                        f"{y / f.height:.5f}"
                        for i, (x, y) in enumerate(poly)]
                fh.write(f"{cls_id} " + " ".join(norm) + "\n")
        if saved < 6 or saved % 200 == 0:                    # debug overlay
            dr = ImageDraw.Draw(img)
            for cls_id, poly in labels:
                dr.polygon(poly, outline=(255, 0, 0) if cls_id == 0
                           else (0, 255, 255))
            img.save(os.path.join(args.out, "debug", stem + ".png"))
        saved += 1

    # capture segments: (hover ENU, target mover, alts)
    seg_plan = [
        ((70.0, -55.0), "mov_1", (5.0, 8.0, 12.0)),    # orange circle rover —
        # stand OFF the circle edge (r=35 @ (70,-100)): mover stays 10–80 m in
        # front instead of passing under the camera (near passes are invisible
        # below the ~21° half-vFOV — the main cause of the target-label famine)
        ((85.0, 10.0), "mov_2", (10.0, 14.0)),         # blue-grey airborne cube
    ]
    await arm_takeoff_goto(s, 6.0)
    for hover, target_name, alts in seg_plan:
        cls_id = kinds[movers[[m["name"] for m in movers].index(target_name)]
                         ["kind"]]
        want = args.per_class - counts[cls_id]
        if want <= 0:
            continue
        per_alt = max(1, want // len(alts))
        for alt in alts:
            got = 0
            await goto_enu(hover[0], hover[1], alt)
            t_end = time.monotonic() + 240
            last_seq = 0
            while got < per_alt and time.monotonic() < t_end:
                tgt = truth.get(target_name)
                me = truth.get(DRONE)
                if tgt and me:
                    yaw, pitch = yaw_pitch(me[3], me[4], me[5], me[6])
                    aim = math.degrees(math.atan2(tgt[0] - me[0],
                                                  tgt[1] - me[1])) % 360
                    try:
                        await goto_enu(hover[0], hover[1], alt, aim)
                    except ActionError:
                        pass
                f = cams.snapshot(0)
                if f is None or f.seq == last_seq:
                    await asyncio.sleep(0.05)
                    continue
                last_seq = f.seq
                if me is None:
                    continue
                yaw, pitch = yaw_pitch(me[3], me[4], me[5], me[6])
                cam = (me[0], me[1], me[2] + CAM_OFFSET[2])
                labels = []
                for m in movers:
                    if truth.get(m["name"]) is None:
                        continue
                    m["_pos"] = truth.get(m["name"])[:3]
                    lab = label_mover(m, cam, yaw, pitch, f.width, f.height)
                    if lab is not None:
                        labels.append((kinds[m["kind"]], lab[1]))
                if not labels:
                    await asyncio.sleep(0.1)
                    continue
                if cls_id not in [c for c, _ in labels]:
                    # the segment's class is out of frame (aim lag, steep
                    # depression on a close pass, sub-6px projection) — do NOT
                    # count this frame toward it (observed: mov_1 segment
                    # "completed" 600 frames with only 210 target labels
                    # because distant mov_2 satisfied the any-label check)
                    await asyncio.sleep(0.1)
                    continue
                save_frame(f, labels)
                got += 1
                counts[cls_id] += sum(1 for c, _ in labels if c == cls_id)
                if got % 50 == 0:
                    print(f"{target_name} alt {alt}: {got}/{per_alt}",
                          flush=True)
                await asyncio.sleep(max(0.0, 1.0 / args.hz - 0.05))
    try:
        await s.action.land()
    except ActionError:
        pass

    with open(os.path.join(args.out, "dataset.yaml"), "w") as fh:
        fh.write(f"path: {args.out}\ntrain: images/train\nval: images/val\n"
                 "names:\n  0: target\n  1: obstacle\n")
    print(f"saved {saved} frames; class counts {counts}", flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
