#!/usr/bin/env python3
"""demo_dataset — W2.5b demo-domain capture + COCO-class auto-labels.

Phase-1 dataset for the demo-domain detector fine-tune (codex R5,
docs/benchmarks/w3-detector-codex-r5.md): ~3,240 vehicle frames (3 meshes x
6 aspects x 3 slant bands x 60), 720 person frames (6x3x40 across both
walkers) and 1,200 hard negatives, labeled from gz ground truth with COCO
class ids (Hatchback/SUV -> car=2, TruckDelivery -> truck=7, walkers ->
person=0; TinyRobot is absent from the demo world — no label invented).
The 80-class COCO head is kept: dataset.yaml carries all 80 names.

Runs INSIDE the w25-capture container against the demo_capture world
(sim/worlds/make_demo_capture_world.py — plain gz server, no PX4). Reuses
scripts/vision_dataset.py machinery (Truth subscriber, convex hull, YOLO-seg
writer shape); the drone/mover capture path of vision_dataset.py is
untouched.

  docker exec w25-capture bash -lc 'cd /workspace && uv run --no-project \
    python scripts/demo_dataset.py --profile vehicles --minutes 30'

Modes:
  (default)   live capture: fill the cell quotas off the camera lattice,
              greedy on (aspect, band, clip-class) with an altitude soft-cap
  --plan      OFFLINE lattice coverage planner (host-safe, no gz): replays
              the analytic trajectories through the exact labeler and tallies
              (cell, clip, alt, split) availability vs quotas. Run this
              BEFORE any boot; it exits 1 on any deficit.
  --qa N      render N annotated samples (polygons drawn) into qa/ for
              eyeball verification.

Projection: EXACT full-3D pinhole (world->cam via the camera basis vectors,
u=cx+f*Xc/Zc) — the same math the W0.1 eval harness validated
(scripts/w0_assets_eval.py:139). vision_dataset.project_point's
tan-of-angle-difference form is only exact for a level camera near
boresight; at 30 deg off-axis it misplaces v by ~26 px and it has no pitch
term for the tilted static cams — same bug class the W0.1 harness caught
(2026-08-01). Labels use the cuboid corners ROTATED by the truth quaternion
(codex R5; vision_dataset's axis-aligned form is wrong for yawed cars by up
to ~0.8 m along the body axis at 45 deg yaw).

Split: 70/15/15 by 20 s sim-time blocks (deterministic, decorrelates
adjacent frames; one continuous boot has no per-run seeds to split by —
codex's "split by capture run" analog).

Capture geometry notes (why the lattice looks like it does): for a level
camera the frame floor sits at the 21.07 deg half-vfov, so bottom-clips
need depression > 21.07 deg (slant-dependent); depression also defines the
elevation aspects (top-down >= 55 deg, oblique 28-55 deg), which makes some
(alt, band) pairs geometrically incompatible with azimuth aspects — the
altitude balance is therefore over the FEASIBLE alts per cell (R5's
4/6/8/10/14 m balance read through slant >= alt and the aspect depression
cuts; verified by --plan).
"""
import argparse
import json
import math
import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_REPO, os.path.join(_REPO, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vision_dataset import FX, FY, CX, CY, hull, yaw_pitch  # noqa: E402

W, H = 640, 360                       # IMX214 capture geometry (make_demo_world)
Z_NEAR = 0.05                         # behind-camera plane rejection (m)

# --- COCO demo-coco profile (codex R5) -------------------------------------
COCO_MAP = {"car_1": 2, "car_2": 2, "car_3": 7,      # SUV -> car per profile
            "walker_1": 0, "walker_2": 0}
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"]

# --- cell spec (codex R5) ---------------------------------------------------
ASPECTS = ("front", "side", "rear", "rear-quarter", "top-down", "oblique")
BANDS_VEH = ((10.0, 15.0), (15.0, 22.0), (22.0, 30.0))
BANDS_PER = ((6.0, 10.0), (10.0, 15.0), (15.0, 22.0))
VEH_QUOTA = {"clean": 36, "bottom": 15, "edge": 20}    # per mesh cell
PER_QUOTA = {"clean": 24, "bottom": 10, "edge": 14}    # per person cell
CELL_HARD = ("clean", "bottom")   # per-cell completion = clean+bottom only;
# edge is a GLOBAL pool (15% of positives, codex R5) — most cells yield few
# edge-clip frames (the yaw_off perp cam produces them), so per-cell edge
# quotas would stall; cell edge caps stay generous and the pool fills where
# geometry allows.
CELL_TOTAL_CAP = 75
EDGE_POOL = {"car_1": 153, "car_2": 153, "car_3": 153, "walkers": 108}
NEG_QUOTA = {"house_1": 200, "house_2": 150, "gas_station": 150,
             "trees": 200, "road": 150, "ground": 250}     # 1,200 total
ALT_CAP_FRAC = 0.6            # no single camera alt > 60% of a cell's quota
DEP_TOPDOWN, DEP_OBLIQUE = 55.0, 28.0     # depression cuts (deg)
SPLIT_BLOCK_S = 20.0          # 20 s blocks: 14 train / 3 val / 3 test
# top-down x band-22-30 is geometrically impossible at the R5 altitudes
# (slant 22-30 m at dep >= 55 deg needs alt >= 18 m) — excluded, see report.
IMPOSSIBLE_CELLS = {("car_1", "top-down", "22-30"),
                    ("car_2", "top-down", "22-30"),
                    ("car_3", "top-down", "22-30")}


def split_of(stamp: float) -> str:
    b = int(stamp // SPLIT_BLOCK_S) % 20
    return "train" if b < 14 else ("val" if b < 17 else "test")


# --- exact camera model -----------------------------------------------------

class StampedTruth:
    """Truth snapshots keyed by SIM STAMP (Pose_V carries a header) — the
    labeler reads the pose AT the frame's sim time, never the latest. The
    latest-vs-frame skew under load (callback batching) measured 0.1-0.3 s,
    which put polygons ~0.5-1.3 m off moving targets (found on the first
    car_1 boot, QA 2026-08-02). Nearest-snapshot <= stamp; at ~50 Hz pose
    updates the residual is <= 8 cm at 4 m/s."""

    def __init__(self, world: str):
        from collections import deque
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        self._snaps = deque(maxlen=512)          # (stamp, {name: pose})
        self._node = GzNode()
        self._node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info",
                             self._on)

    def _on(self, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        models = {}
        for p in msg.pose:
            models[p.name] = (p.position.x, p.position.y, p.position.z,
                              p.orientation.w, p.orientation.x,
                              p.orientation.y, p.orientation.z)
        self._snaps.append((s, models))

    def at(self, stamp: float) -> dict:
        """Pose table nearest to `stamp` (<= stamp preferred)."""
        best = None
        for s, m in self._snaps:                 # chronological; keep last <=
            if s <= stamp + 1e-9:
                best = (s, m)
            else:
                break
        if best is None:
            best = self._snaps[0] if self._snaps else (0.0, {})
        return best[1]

    def latest(self) -> dict:
        return self._snaps[-1][1] if self._snaps else {}


class Cam:
    """Static pitched camera: SDF pose (x,y,z,roll=0,pitch_down,yaw) with the
    make_demo_world convention (pitch+ tilts the +X optical axis DOWN).
    Basis: f=optical axis, r=image right, d=image down (world ENU)."""

    def __init__(self, x, y, z, pitch, yaw):
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.pos = (x, y, z)
        self.f = (cp * cy, cp * sy, -sp)
        # r = norm(f x up) = (f_y, -f_x, 0); level-east check: f=(1,0,0) ->
        # r=(0,-1,0) (south is right when facing east). d = f x r (level:
        # (0,0,-1) = down).
        f = self.f
        r = (f[1], -f[0], 0.0)
        n = math.hypot(r[0], r[1])
        if n < 1e-9:                # looking straight down: degenerate
            r = (0.0, -1.0, 0.0)
            n = 1.0
        self.r = (r[0] / n, r[1] / n, r[2] / n)
        rr = self.r
        self.d = (f[1] * rr[2] - f[2] * rr[1],
                  f[2] * rr[0] - f[0] * rr[2],
                  f[0] * rr[1] - f[1] * rr[0])

    def project(self, px, py, pz):
        """World point -> (u, v) or None when behind the near plane."""
        vx, vy, vz = (px - self.pos[0], py - self.pos[1], pz - self.pos[2])
        zc = vx * self.f[0] + vy * self.f[1] + vz * self.f[2]
        if zc < Z_NEAR:
            return None
        xc = vx * self.r[0] + vy * self.r[1] + vz * self.r[2]
        yc = vx * self.d[0] + vy * self.d[1] + vz * self.d[2]
        return (CX + FX * xc / zc, CY + FY * yc / zc)


def _rot(qw, qx, qy, qz, x, y, z):
    """Rotate vector (x,y,z) by quaternion (w,x,y,z) — active rotation."""
    # t = 2 q_vec x v
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx))


def mover_corners(pos, quat, shape):
    """8 world corners of the mover's shape box, ROTATED by the truth quat
    and lifted to the collision-box center (z + h/2 + 0.05, make_demo_world)."""
    x0, y0, z0 = pos
    qw, qx, qy, qz = quat
    cx, cy, cz = x0, y0, z0 + shape["h"] / 2.0 + 0.05
    out = []
    for dx in (-shape["w"] / 2, shape["w"] / 2):
        for dy in (-shape["d"] / 2, shape["d"] / 2):
            for dz in (-shape["h"] / 2, shape["h"] / 2):
                rx, ry, rz = _rot(qw, qx, qy, qz, dx, dy, dz)
                out.append((cx + rx, cy + ry, cz + rz))
    return (cx, cy, cz), out


def aspect_of(phi_deg: float, dep_deg: float) -> str:
    """phi: |bearing of the camera FROM the target - target heading| (deg,
    0 = camera dead ahead). dep: depression of the target from the camera."""
    if dep_deg >= DEP_TOPDOWN:
        return "top-down"
    if dep_deg >= DEP_OBLIQUE:
        return "oblique"
    a = abs(phi_deg)
    if a <= 45.0:
        return "front"
    if a <= 100.0:
        return "side"
    if a <= 150.0:
        return "rear-quarter"
    return "rear"


def band_of(slant: float, bands) -> str | None:
    for i, (lo, hi) in enumerate(bands):
        if lo <= slant < hi:
            return f"{int(lo)}-{int(hi)}"
    return None


def label_mover(cam: Cam, pos, quat, shape):
    """-> (poly, clip) or None. clip in {clean, bottom, edge} from the
    clamped hull (bottom takes precedence over horizontal edge)."""
    _, corners = mover_corners(pos, quat, shape)
    pts = []
    for c in corners:
        pr = cam.project(c[0], c[1], c[2])
        if pr is not None:
            pts.append(pr)
    if len(pts) < 3:
        return None
    poly = hull(pts)
    poly = [(min(max(x, 0.0), W), min(max(y, 0.0), H)) for x, y in poly]
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    if max(xs) - min(xs) < 6 or max(ys) - min(ys) < 6:
        return None                       # sub-pixel-ish: useless as a label
    clip = "clean"
    if max(ys) >= H - 0.5:
        clip = "bottom"
    elif min(xs) <= 0.5 or max(xs) >= W - 0.5:
        clip = "edge"
    return poly, clip


def frame_labels(cam: Cam, movers: list[dict], truth_get):
    """All visible mover labels for one frame -> list of dicts (poly, clip,
    cell key parts, slant/dep/phi). `truth_get(name)` -> (x,y,z,qw,qx,qy,qz).
    """
    out = []
    for m in movers:
        t = truth_get(m["name"])
        if t is None:
            continue
        pos, quat = t[:3], t[3:]
        lab = label_mover(cam, pos, quat, m["shape"])
        if lab is None:
            continue
        poly, clip = lab
        center = (pos[0], pos[1], pos[2] + m["shape"]["h"] / 2.0 + 0.05)
        dx, dy = cam.pos[0] - center[0], cam.pos[1] - center[1]
        horiz = math.hypot(dx, dy)
        slant = math.sqrt(horiz * horiz + (cam.pos[2] - center[2]) ** 2)
        dep = math.degrees(math.atan2(cam.pos[2] - center[2], horiz))
        bearing, _ = yaw_pitch(*quat)            # heading (0=N cw+)
        cam_bearing = math.degrees(math.atan2(dx, dy))
        phi = (cam_bearing - math.degrees(bearing) + 180.0) % 360.0 - 180.0
        person = m["name"].startswith("walker")
        asp = aspect_of(phi, dep)
        band = band_of(slant, BANDS_PER if person else BANDS_VEH)
        cell = ("person" if person else m["name"], asp, band)
        out.append({"mover": m["name"], "cls": COCO_MAP[m["name"]],
                    "poly": poly, "clip": clip, "cell": cell,
                    "slant": slant, "dep": dep, "phi": phi,
                    "alt": round(cam.pos[2])})
    return out


# --- quota binder -----------------------------------------------------------

class Binder:
    """Greedy (cell, clip) quota fill + altitude soft-cap. Vehicle cells are
    per-mesh (distinct visual domains); person cells are walker-agnostic."""

    def __init__(self, state: dict | None = None, feas: dict | None = None):
        s = state or {}
        self.counts: dict = s.get("counts", {})        # cellkey|clip -> n
        self.altcounts: dict = s.get("altcounts", {})  # cellkey|alt -> n
        self.neg: dict = s.get("neg", {})              # scene -> n
        self.edge_frames: int = s.get("edge_frames", 0)
        self.saved: int = s.get("saved", 0)
        # replay altitude counts per cell key (group|aspect|band) -> {alt: n}
        # — from a quick analytic replay at capture start; the needs() cap
        # floors each alt at its share of the cell (default: plain 60% cap)
        self.feas: dict = feas or {}

    @staticmethod
    def _key(cell, clip):
        return "|".join(str(c) for c in cell) + "|" + clip

    def needs(self, lab: dict) -> bool:
        cell, clip = lab["cell"], lab["clip"]
        if cell[2] is None:                     # out of all slant bands
            return False
        quota = (PER_QUOTA if cell[0] == "person" else VEH_QUOTA)[clip]
        if self.counts.get(self._key(cell, clip), 0) >= quota:
            return False
        total = sum(self.counts.get(self._key(cell, c), 0)
                    for c in ("clean", "bottom", "edge"))
        if total >= CELL_TOTAL_CAP:
            return False
        qsum = sum((PER_QUOTA if cell[0] == "person" else VEH_QUOTA).values())
        # alt soft-cap: <=60% of the cell quota per camera altitude — but
        # never below the alt's REPLAY SHARE of the cell, else effectively
        # single-alt cells (most steep/pitched-cam cells) cap out below quota
        # and the boot idles forever on "open" cells (car_2 boot 2-5 stall,
        # 2026-08-02). feas maps cell key -> {alt: replay count}.
        alts = self.feas.get("|".join(str(c) for c in cell)) or {}
        share = (alts.get(lab["alt"], 0) / sum(alts.values())) if alts else 0.0
        cap = math.ceil(qsum * max(ALT_CAP_FRAC, share))
        akey = "|".join(str(c) for c in cell) + f"|alt{lab['alt']}"
        return self.altcounts.get(akey, 0) < cap

    def record(self, lab: dict) -> None:
        cell, clip = lab["cell"], lab["clip"]
        if cell[2] is None:
            return
        k = self._key(cell, clip)
        self.counts[k] = self.counts.get(k, 0) + 1
        # the alt budget (needs() gate) is spent ONLY by in-quota fills: a
        # saved frame records every visible label, so clips already at quota
        # (bottom overshoot, e.g. 33/10) would otherwise consume the cell's
        # alt budget and deadlock the still-open clips (person rear-quarter/
        # oblique cells, walkers boot 2026-08-02)
        quota = (PER_QUOTA if cell[0] == "person" else VEH_QUOTA)[clip]
        if self.counts[k] <= quota:
            akey = "|".join(str(c) for c in cell) + f"|alt{lab['alt']}"
            self.altcounts[akey] = self.altcounts.get(akey, 0) + 1

    def filled(self, profile: str) -> bool:
        """Profile-scoped completion: each vehicle boot fills its own mesh's
        cells (clean+bottom per cell + the global edge pool), walkers fills
        the person cells, negatives the neg scenes."""
        def cells_open(name, bands, quota):
            for asp in ASPECTS:
                for band in (f"{int(a)}-{int(b)}" for a, b in bands):
                    if (name, asp, band) in IMPOSSIBLE_CELLS:
                        continue
                    for clip in CELL_HARD:
                        k = f"{name}|{asp}|{band}|{clip}"
                        if self.counts.get(k, 0) < quota[clip]:
                            return True
            return False

        if profile in ("car_1", "car_2", "car_3"):
            if cells_open(profile, BANDS_VEH, VEH_QUOTA):
                return False
            return self.edge_frames >= EDGE_POOL[profile]
        if profile == "walkers":
            if cells_open("person", BANDS_PER, PER_QUOTA):
                return False
            return self.edge_frames >= EDGE_POOL[profile]
        if profile == "negatives":
            return all(self.neg.get(s, 0) >= q for s, q in NEG_QUOTA.items())
        return False

    def state(self) -> dict:
        return {"counts": self.counts, "altcounts": self.altcounts,
                "neg": self.neg, "edge_frames": self.edge_frames,
                "saved": self.saved}


NEG_SCENES = tuple(NEG_QUOTA)


def cells_tally(binder: Binder) -> dict:
    """-> {(group, aspect, band): {clip: n}} for reporting."""
    out: dict = {}
    for k, n in binder.counts.items():
        g, asp, band, clip = k.split("|")
        out.setdefault((g, asp, band), {})[clip] = n
    return out


# --- planner (host-safe, no gz) ----------------------------------------------

def plan(profile: str, seconds: float, dt: float = 0.2) -> int:
    from agents.world.trajectory import pos_xy, vel_xy
    from sim.worlds.make_demo_capture_world import CAPTURE_CAMERAS

    from sim.worlds.make_demo_world import MOVERS

    cams_spec = CAPTURE_CAMERAS[profile] if profile != "all" else \
        [c for cs in CAPTURE_CAMERAS.values() for c in cs]
    cams = [(c["name"], Cam(*c["pose"][0:3], c["pose"][4], c["pose"][5]))
            for c in cams_spec]

    def truth_at(t):
        table = {}
        for m in MOVERS:
            x, y = pos_xy(m["traj"], t)
            vx, vy = vel_xy(m["traj"], t)
            ye = math.atan2(vy, vx)               # ENU yaw from +E
            table[m["name"]] = (x, y, m["z"],
                                math.cos(ye / 2), 0.0, 0.0, math.sin(ye / 2))
        return table

    avail: dict = {}        # (cell, clip) -> {n, alts, splits}
    t = 0.0
    while t < seconds:
        table = truth_at(t)
        for cname, cam in cams:
            for lab in frame_labels(cam, MOVERS, table.get):
                key = (lab["cell"][0], lab["cell"][1], lab["cell"][2],
                       lab["clip"])
                a = avail.setdefault(key, {"n": 0, "alts": set(),
                                           "splits": set()})
                a["n"] += 1
                a["alts"].add(lab["alt"])
                a["splits"].add(split_of(t))
        t += dt
    # report vs quotas (scoped to the profile's own cells: car profiles check
    # only their mesh; walkers checks the walker-agnostic person cells;
    # negatives has no mover cells to verify)
    deficits = []
    if profile in ("car_1", "car_2", "car_3"):
        groups = ((profile, BANDS_VEH, VEH_QUOTA),)
    elif profile == "walkers":
        groups = (("person", BANDS_PER, PER_QUOTA),)
    elif profile == "negatives":
        print(f"planner: negatives profile — {len(cams)} scene cams, no mover "
              "cells to verify (quotas fill from mover-free frames)")
        return 0
    else:
        groups = (("car_1", BANDS_VEH, VEH_QUOTA),
                  ("car_2", BANDS_VEH, VEH_QUOTA),
                  ("car_3", BANDS_VEH, VEH_QUOTA),
                  ("person", BANDS_PER, PER_QUOTA))
    print(f"planner: profile={profile} {seconds:.0f}s @ {1/dt:.0f} Hz, "
          f"{len(cams)} cams")
    print(f"{'cell':42s} {'need':>16s} {'avail':>16s} alts splits")
    for g, bands, quota in groups:
        for asp in ASPECTS:
            for band in (f"{int(a)}-{int(b)}" for a, b in bands):
                if (g, asp, band) in IMPOSSIBLE_CELLS:
                    continue
                for clip in CELL_HARD:
                    q = quota[clip]
                    key = (g, asp, band, clip)
                    a = avail.get(key, {"n": 0, "alts": set(),
                                        "splits": set()})
                    flag = "OK " if a["n"] >= q * 1.5 else "LOW"
                    if a["n"] < q * 1.3:
                        deficits.append((key, a["n"], q))
                    if clip == "clean" or flag == "LOW":
                        print(f"{g + '|' + asp + '|' + band:42s} "
                              f"{clip + ':' + str(q):>16s} {a['n']:>16d} "
                              f"{sorted(a['alts'])} {sorted(a['splits'])}"
                              f"  {flag}")
    # edge pool: global availability (banded cells only)
    pool = EDGE_POOL.get(profile, 0)
    if pool:
        edges = sum(a["n"] for (g, asp, band, clip), a in avail.items()
                    if clip == "edge" and band is not None
                    and (profile == "walkers") == (g == "person"))
        print(f"edge pool: {edges} avail vs target {pool} "
              f"({'OK' if edges >= pool * 1.3 else 'LOW'})")
        if edges < pool * 1.3:
            deficits.append((("edge-pool",), edges, pool))
    if deficits:
        print(f"\nDEFICITS ({len(deficits)}): cells under 1.3x quota")
        for key, n, q in deficits[:40]:
            print(f"  {key}: {n} avail vs {q} needed")
        return 1
    print("\nno deficits >=1.3x quota — lattice is sufficient")
    return 0


# --- live capture (container only) -------------------------------------------

def _write_yaml(out: str) -> None:
    names = "\n".join(f"  {i}: {n}" for i, n in enumerate(COCO_NAMES))
    with open(os.path.join(out, "dataset.yaml"), "w") as fh:
        fh.write(f"path: {out}\ntrain: images/train\nval: images/val\n"
                 f"test: images/test\nnames:\n{names}\n")


def capture(args) -> int:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.image_pb2 import Image as GzImage
    from PIL import Image as PILImage

    cfg = json.load(open(args.sidecar))
    movers = [m for m in cfg["movers"] if m["name"] in COCO_MAP]
    cams_spec = cfg["cameras"]
    for d in ("images/train", "images/val", "images/test",
              "labels/train", "labels/val", "labels/test", "qa"):
        os.makedirs(os.path.join(args.out, d), exist_ok=True)

    state_path = os.path.join(args.out, "capture_state.json")
    state = None
    if os.path.exists(state_path) and not args.fresh:
        state = json.load(open(state_path))
        print(f"resuming: {state.get('saved', 0)} frames already saved",
              flush=True)
    # feasible altitude SHARE per cell (alt soft-cap floor, Binder.needs):
    # quick analytic replay of the profile lattice, same math as --plan
    # (heading from the trajectory velocity — an identity quat misclassifies
    # the aspect and pollutes the alt shares, car_2 stall 2026-08-02)
    from agents.world.trajectory import pos_xy, vel_xy
    feas_alts: dict = {}
    _t = 0.0
    while _t < 300.0:
        _tbl = {}
        for m in movers:
            _x, _y = pos_xy(m["traj"], _t)
            _vx, _vy = vel_xy(m["traj"], _t)
            _ye = math.atan2(_vy, _vx)
            _tbl[m["name"]] = (_x, _y, m["z"],
                               math.cos(_ye / 2), 0.0, 0.0, math.sin(_ye / 2))
        for c in cams_spec:
            _cam = Cam(*c["pose"][0:3], c["pose"][4], c["pose"][5])
            for lab in frame_labels(_cam, movers, _tbl.get):
                if lab["cell"][2] is not None:
                    _k = "|".join(str(x) for x in lab["cell"])
                    _alts = feas_alts.setdefault(_k, {})
                    _alts[lab["alt"]] = _alts.get(lab["alt"], 0) + 1
        _t += 0.5
    feas = feas_alts
    binder = Binder(state, feas)

    truth = StampedTruth(cfg.get("world", "demo_capture"))
    cams = {c["name"]: (Cam(*c["pose"][0:3], c["pose"][4], c["pose"][5]), c)
            for c in cams_spec}
    node = GzNode()
    world = cfg.get("world", "demo_capture")
    jsonl = open(os.path.join(args.out, "frames.jsonl"), "a")
    last_neg: dict = {}
    t0 = time.monotonic()
    deadline = t0 + args.minutes * 60.0
    done = {"flag": False}

    def save_frame(msg, cam_name, labels, negative):
        split = split_of(msg.header.stamp.sec
                         + msg.header.stamp.nsec * 1e-9)
        binder.saved += 1
        stem = f"{'neg' if negative else 'gz'}_{binder.saved:05d}"
        img = PILImage.frombytes("RGB", (msg.width, msg.height),
                                 bytes(msg.data))
        img.save(os.path.join(args.out, "images", split, stem + ".png"))
        with open(os.path.join(args.out, "labels", split, stem + ".txt"),
                  "w") as fh:
            for lab in labels:
                norm = [f"{v:.5f}" for x, y in lab["poly"]
                        for v in (x / msg.width, y / msg.height)]
                fh.write(f"{lab['cls']} " + " ".join(norm) + "\n")
        jsonl.write(json.dumps({
            "stem": stem, "split": split, "cam": cam_name,
            "stamp": round(msg.header.stamp.sec
                           + msg.header.stamp.nsec * 1e-9, 2),
            "negative": negative,
            "labels": [{"cls": l["cls"], "mover": l["mover"],
                        "cell": list(l["cell"]), "clip": l["clip"],
                        "slant": round(l["slant"], 1), "alt": l["alt"],
                        "phi": round(l["phi"]), "dep": round(l["dep"], 1)}
                       for l in labels]}) + "\n")
        jsonl.flush()

    def make_cb(cam_name: str):
        cam, spec = cams[cam_name]
        scene = spec["target"].split(":", 1)[1] if \
            spec["target"].startswith("neg:") else None

        def cb(msg):
            if done["flag"] or time.monotonic() > deadline:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            labels = frame_labels(cam, movers, truth.at(stamp).get)
            if labels:
                need = [l for l in labels if binder.needs(l)]
                if not need:
                    return
                for l in labels:
                    binder.record(l)
                if any(l["clip"] == "edge" for l in labels):
                    binder.edge_frames += 1
                save_frame(msg, cam_name, labels, negative=False)
            elif scene is not None:
                if binder.neg.get(scene, 0) >= NEG_QUOTA[scene]:
                    return
                now = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                if now - last_neg.get(cam_name, -1e9) < args.neg_interval:
                    return
                last_neg[cam_name] = now
                binder.neg[scene] = binder.neg.get(scene, 0) + 1
                save_frame(msg, cam_name, [], negative=True)
        return cb

    cbs = []
    for cname in cams:
        topic = (f"/world/{world}/model/{cname}/link/link"
                 f"/sensor/IMX214/image")
        cb = make_cb(cname)
        cbs.append(cb)                        # keep alive for gz
        node.subscribe(GzImage, topic, cb)
    print(f"subscribed {len(cams)} cams, world={world}, "
          f"deadline {args.minutes} min", flush=True)

    last_report = 0.0
    profile = cfg.get("profile", "car_1")
    while time.monotonic() < deadline:
        time.sleep(2.0)
        if binder.filled(profile):
            print("all quotas FILLED", flush=True)
            break
        if time.monotonic() - last_report > 30:
            last_report = time.monotonic()
            tally = cells_tally(binder)
            open_cells = sum(1 for (g, a, b), cl in tally.items()
                             for c, q in (PER_QUOTA if g == "person"
                                          else VEH_QUOTA).items()
                             if cl.get(c, 0) < q)
            print(f"t+{time.monotonic() - t0:.0f}s saved={binder.saved} "
                  f"cells-with-open-quotas={open_cells} neg={binder.neg}",
                  flush=True)
            with open(state_path, "w") as fh:
                json.dump(binder.state(), fh)
    done["flag"] = True
    with open(state_path, "w") as fh:
        json.dump(binder.state(), fh)
    jsonl.close()
    _write_yaml(args.out)

    tally = cells_tally(binder)
    summary = {"profile": cfg.get("profile"), "saved": binder.saved,
               "neg": binder.neg,
               "cells": {"|".join(k): v for k, v in tally.items()},
               "alts": binder.altcounts}
    with open(os.path.join(args.out, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=2)
    print(f"saved {binder.saved} frames total", flush=True)
    sys.stdout.flush()
    os._exit(0)


# --- QA renderer --------------------------------------------------------------

def qa(args) -> int:
    from PIL import Image as PILImage, ImageDraw

    recs = [json.loads(l) for l in
            open(os.path.join(args.out, "frames.jsonl"))]
    COLORS = {0: (80, 200, 80), 2: (240, 80, 60), 7: (240, 200, 60)}
    picked: dict = {}
    mover = getattr(args, "qa_mover", None)

    def want(rec):
        if rec["negative"]:
            return [("neg", rec["cam"])] if mover in (None, "neg") else []
        if mover == "neg":
            return []
        out = []
        for l in rec["labels"]:
            if mover and l["mover"] != mover:
                continue
            out.append((COCO_NAMES[l["cls"]], l["cell"][1]))
            if l["clip"] != "clean":
                out.append(("clip", l["clip"]))
        return out

    # filtered picks walk newest-first so QA covers the LATEST boot's frames
    for rec in (reversed(recs) if mover else iter(recs)):
        for k in want(rec):
            picked.setdefault(k, rec)
    # ordering: all aspects of car/truck/person, clips, negatives
    order = [k for k in picked if k[0] in ("car", "truck", "person")]
    order += [k for k in picked if k[0] == "clip"]
    order += [k for k in picked if k[0] == "neg"]
    keys = order[: args.qa]
    os.makedirs(os.path.join(args.out, "qa"), exist_ok=True)
    for k in keys:
        rec = picked[k]
        p = os.path.join(args.out, "images", rec["split"], rec["stem"] + ".png")
        img = PILImage.open(p).convert("RGB")
        dr = ImageDraw.Draw(img)
        lp = os.path.join(args.out, "labels", rec["split"], rec["stem"] + ".txt")
        meta = {l["cls"]: l for l in rec["labels"]}
        if os.path.exists(lp):
            for line in open(lp):
                parts = line.split()
                cls = int(parts[0])
                coords = [float(v) for v in parts[1:]]
                poly = [(coords[i] * img.width, coords[i + 1] * img.height)
                        for i in range(0, len(coords), 2)]
                col = COLORS.get(cls, (255, 255, 255))
                dr.polygon(poly, outline=col, width=2)
                m = meta.get(cls, {})
                dr.text((poly[0][0] + 3, poly[0][1] + 3),
                        f"{COCO_NAMES[cls]} {m.get('cell', ['', '?', '?'])[1]}"
                        f" {m.get('cell', ['', '', '?'])[2]}"
                        f" {m.get('clip', '')}", fill=col)
        tag = "-".join(k)
        dr.text((6, 6), f"{rec['stem']} {rec['cam']} {tag}",
                fill=(255, 255, 255))
        img.save(os.path.join(args.out, "qa", f"qa_{rec['stem']}_{tag}.png"))
        print("qa:", rec["stem"], k, flush=True)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sidecar", default="/workspace/PX4-Autopilot/Tools/"
                    "simulation/gz/worlds/demo_capture_boxes.json")
    ap.add_argument("--out", default="/workspace/evals/out/w25b_dataset")
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--neg-interval", type=float, default=1.5,
                    help="min sim-seconds between saved negatives per cam")
    ap.add_argument("--fresh", action="store_true",
                    help="ignore capture_state.json (start quotas over)")
    ap.add_argument("--plan", action="store_true",
                    help="offline lattice coverage check (host-safe)")
    ap.add_argument("--profile", default=os.environ.get("CAPTURE_PROFILE",
                                                        "vehicles"))
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--qa", type=int, default=0,
                    help="render N annotated samples into qa/ and exit")
    ap.add_argument("--qa-mover", default=None,
                    help="restrict --qa picks to this mover (e.g. car_2, "
                         "walker_1) or 'neg' for negative scenes only")
    args = ap.parse_args()

    if args.plan:
        return plan(args.profile, args.seconds)
    if args.qa:
        return qa(args)
    return capture(args)


if __name__ == "__main__":
    sys.exit(main())
