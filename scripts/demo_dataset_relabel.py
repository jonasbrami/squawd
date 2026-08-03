#!/usr/bin/env python3
"""demo_dataset_relabel — repair W2.5b label polygons scrambled by the
save_frame writer bug (demo_dataset.py, fixed 2026-08-02): the buggy writer
emitted ONE number per poly vertex (x/w for even vertex indices, y/h for
odd), so every 6-vertex box-projection hull landed in the label file as
6 mixed-coordinate numbers. All live hulls are 6-vertex (a box projection
is a zonogon, <= 2x3 vertices), so every pre-fix label line has exactly
6 numbers.

Repair: per label line, recover the LIVE polygon by fitting the box pose
(dx, dy, dyaw about the analytic trajectory pose at the frame stamp —
agents.world.trajectory replay, same as --plan) so that its projected
6-vertex hull, passed through the buggy writer pattern, reproduces the
6 recorded numbers. 6 residuals (clamped-at-frame-edge coordinates carry
no equality information and are dropped), 3 params, hand-rolled damped
Gauss-Newton (no numpy in .venv). The analytic init sits within the mover
plugin's snap tolerance of live truth (velocity-driven + drift snap), so
the fit only has to absorb the snap-level position error and the
heading_align slew at corners.

Closed-loop verification: the fitted poly is re-scrambled with the buggy
pattern and compared to the original file bytes (2e-4 ~ rounding); only
verified labels are written. Labels that cannot be verified (fit failure,
rare hull-order flips) fall back to the analytic-regenerated poly and are
counted as "approx". Post-fix frames (full polygons, >6 numbers) are left
untouched.

  .venv/bin/python scripts/demo_dataset_relabel.py \
      --out evals/out/w25b_dataset [--write]
"""
import argparse
import json
import math
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_REPO, os.path.join(_REPO, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from agents.world.trajectory import pos_xy, vel_xy              # noqa: E402
from sim.worlds.make_demo_capture_world import CAPTURE_CAMERAS  # noqa: E402
from sim.worlds.make_demo_world import MOVERS                   # noqa: E402
from demo_dataset import Cam, COCO_MAP, W, H, _rot              # noqa: E402

VERIFY_TOL = 2e-4          # closed-loop re-scramble match (normalized)
RMS_ACCEPT = 1.5           # px, weighted residual RMS acceptance


def cam_lookup():
    cams = {}
    for profile, spec in CAPTURE_CAMERAS.items():
        for c in spec:
            if c["name"] in cams and cams[c["name"]]["pose"] != c["pose"]:
                raise SystemExit(f"cam name collision: {c['name']}")
            cams[c["name"]] = c
    return cams


def truth_at(t):
    table = {}
    for m in MOVERS:
        x, y = pos_xy(m["traj"], t)
        vx, vy = vel_xy(m["traj"], t)
        ye = math.atan2(vy, vx)
        table[m["name"]] = (x, y, m["z"],
                            math.cos(ye / 2), 0.0, 0.0, math.sin(ye / 2))
    return table


def buggy_scramble(poly):
    """What the buggy writer emitted for a poly: x/w for even i, y/h odd."""
    return [x / W if i % 2 == 0 else y / H for i, (x, y) in enumerate(poly)]


def norm_poly(poly):
    return [v for x, y in poly for v in (x / W, y / H)]


def hull_idx(points):
    """Monotone-chain hull over [(x, y), ...] -> vertex INDICES, CCW,
    matching vision_dataset.hull's ordering (sorted-set, lower+upper)."""
    order = sorted(range(len(points)), key=lambda i: points[i])
    pts = [points[i] for i in order]
    idx = list(order)
    # dedup identical points (keep first index)
    up, ui = [pts[0]], [idx[0]]
    for p, i in zip(pts[1:], idx[1:]):
        if p != up[-1]:
            up.append(p)
            ui.append(i)
    pts, idx = up, ui
    if len(pts) <= 2:
        return idx

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))
    lower, li = [], []
    for p, i in zip(pts, idx):
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
            li.pop()
        lower.append(p)
        li.append(i)
    upper, ui2 = [], []
    for p, i in zip(reversed(pts), reversed(idx)):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
            ui2.pop()
        upper.append(p)
        ui2.append(i)
    return li[:-1] + ui2[:-1]


def project_box(cam, x0, y0, z0, yaw, shape):
    """8 corners of the yaw-rotated box -> (hull poly in hull order) or
    None when any hull corner is behind the near plane. Mirrors
    demo_dataset.mover_corners for pure-yaw poses."""
    cz = z0 + shape["h"] / 2.0 + 0.05
    qw, qx, qy, qz = math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)
    corners = []
    for dx in (-shape["w"] / 2, shape["w"] / 2):
        for dy in (-shape["d"] / 2, shape["d"] / 2):
            for dz in (-shape["h"] / 2, shape["h"] / 2):
                rx, ry, rz = _rot(qw, qx, qy, qz, dx, dy, dz)
                corners.append((x0 + rx, y0 + ry, cz + rz))
    pts = []
    for c in corners:
        pr = cam.project(c[0], c[1], c[2])
        pts.append(pr if pr is not None else (float("nan"), float("nan")))
    hi = hull_idx(pts)
    poly = []
    for i in hi:
        if pts[i][0] != pts[i][0]:      # NaN: corner behind camera
            return None
        poly.append(pts[i])
    return poly


def _residuals(poly, known):
    """known: [(value, kind)] with kind 'x' or 'y' per hull vertex (the
    buggy pattern) — the writer's alternating coordinate readout. Frame-
    edge-clamped knowns (== 0/W/H) carry only an inequality: dropped."""
    r, wts = [], []
    for i, (val, kind) in enumerate(known):
        u, v = poly[i]
        if kind == "x":
            clamped = val <= 0.5 or val >= W - 0.5
            r.append(u - val)
        else:
            clamped = val <= 0.5 or val >= H - 0.5
            r.append(v - val)
        wts.append(0.0 if clamped else 1.0)
    return r, wts


def fit_poly(cam, shape, x0, y0, z0, yaw0, known):
    """Damped Gauss-Newton on (dx, dy, dyaw); known = 6 (value, kind).
    -> (poly, rms) or None."""
    p = [0.0, 0.0, 0.0]
    eps = (0.02, 0.02, 0.002)
    lam = 1e-3

    def ev(pp):
        pl = project_box(cam, x0 + pp[0], y0 + pp[1], z0, yaw0 + pp[2],
                         shape)
        return pl if pl is not None and len(pl) == len(known) else None

    poly = ev(p)
    if poly is None or len(poly) != len(known):
        return None
    r, wts = _residuals(poly, known)
    if sum(wts) < 4:
        return None

    def cost(rr):
        return sum(w * x * x for w, x in zip(wts, rr))

    c0 = cost(r)
    for _ in range(40):
        # numeric jacobian
        J = []
        for k in range(3):
            pp = list(p)
            pp[k] += eps[k]
            pk = ev(pp)
            if pk is None:
                return None
            rk, _ = _residuals(pk, known)
            J.append([(rk[i] - r[i]) / eps[k] for i in range(len(r))])
        # normal equations (weighted): A = Jw^T J, g = Jw^T r
        A = [[sum(wts[i] * J[a][i] * J[b][i] for i in range(len(r)))
              for b in range(3)] for a in range(3)]
        g = [sum(wts[i] * J[a][i] * r[i] for i in range(len(r)))
             for a in range(3)]
        for a in range(3):
            A[a][a] += lam
        # solve 3x3 by gaussian elimination
        M = [A[0] + [g[0]], A[1] + [g[1]], A[2] + [g[2]]]
        try:
            for col in range(3):
                piv = max(range(col, 3), key=lambda rr_: abs(M[rr_][col]))
                if abs(M[piv][col]) < 1e-12:
                    return None
                M[col], M[piv] = M[piv], M[col]
                for rr_ in range(col + 1, 3):
                    f = M[rr_][col] / M[col][col]
                    for cc in range(col, 4):
                        M[rr_][cc] -= f * M[col][cc]
            step = [0.0] * 3
            for col in reversed(range(3)):
                s = M[col][3] - sum(M[col][cc] * step[cc]
                                    for cc in range(col + 1, 3))
                step[col] = s / M[col][col]
        except ZeroDivisionError:
            return None
        pn = [p[i] - step[i] for i in range(3)]
        poln = ev(pn)
        if poln is None:
            return None
        rn, _ = _residuals(poln, known)
        cn = cost(rn)
        if cn < c0:
            p, poly, r, c0 = pn, poln, rn, cn
            lam = max(lam / 3, 1e-9)
        else:
            lam *= 5
            if lam > 1e6:
                break
    neff = sum(wts)
    rms = math.sqrt(c0 / neff) if neff else float("inf")
    return (poly, rms) if rms <= RMS_ACCEPT else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="evals/out/w25b_dataset")
    ap.add_argument("--write", action="store_true",
                    help="overwrite label files (default: dry run)")
    args = ap.parse_args()
    out = args.out

    cams = cam_lookup()
    movers = {m["name"]: m for m in MOVERS if m["name"] in COCO_MAP}
    recs = [json.loads(l) for l in open(os.path.join(out, "frames.jsonl"))]

    n = dict(fit=0, approx=0, direct=0, neg=0, nocam=0, badshape=0)
    approx_examples = []
    for rec in recs:
        if rec["negative"]:
            n["neg"] += 1
            continue
        spec = cams.get(rec["cam"])
        if spec is None:
            n["nocam"] += 1
            continue
        cam = Cam(*spec["pose"][0:3], spec["pose"][4], spec["pose"][5])
        table = truth_at(rec["stamp"])
        lp = os.path.join(out, "labels", rec["split"], rec["stem"] + ".txt")
        old_lines = [l.split() for l in open(lp)] if os.path.exists(lp) else []
        if len(old_lines) != len(rec["labels"]):
            n["badshape"] += 1
            continue
        new_lines = []
        for old, meta in zip(old_lines, rec["labels"]):
            m = movers[meta["mover"]]
            t = table[m["name"]]
            yaw0 = 2.0 * math.atan2(t[6], t[3])
            nums = [float(v) for v in old[1:]]
            poly = None
            if len(nums) == 6:
                # scrambled pre-fix line (alternating coords of a 6-vertex
                # hull) vs post-fix 3-vertex sliver (clamped corner stub):
                # the scrambled pseudo-triangle spans the target (>= ~1k
                # px^2), a true sliver is a tiny corner artifact
                pts3 = [(nums[i] * W, nums[i + 1] * H)
                        for i in (0, 2, 4)]
                area3 = abs((pts3[1][0] - pts3[0][0])
                            * (pts3[2][1] - pts3[0][1])
                            - (pts3[2][0] - pts3[0][0])
                            * (pts3[1][1] - pts3[0][1])) / 2.0
                if area3 < 200.0:
                    n["direct"] += 1
                    new_lines.append(" ".join(old))
                    continue
                known = [(nums[i] * (W if i % 2 == 0 else H),
                          "x" if i % 2 == 0 else "y") for i in range(6)]
                fit = fit_poly(cam, m["shape"], t[0], t[1], t[2], yaw0,
                               known)
                if fit is not None:
                    # the live writer saved the FRAME-CLAMPED poly — clamp
                    # the fitted poly the same way before verifying/writing
                    cand = [(min(max(x, 0.0), W), min(max(y, 0.0), H))
                            for x, y in fit[0]]
                    scr = buggy_scramble(cand)
                    if all(abs(a - b) <= VERIFY_TOL
                           for a, b in zip(scr, nums)):
                        poly = cand
                if poly is not None:
                    n["fit"] += 1
                else:
                    n["approx"] += 1
                    if len(approx_examples) < 8:
                        approx_examples.append((rec["stem"], meta["mover"],
                                                rec["cam"], rec["stamp"]))
            if poly is None and len(nums) != 6:
                n["direct"] += 1        # post-fix full polygon: keep bytes
                new_lines.append(" ".join(old))
                continue
            if poly is None:            # fallback: analytic-regenerated
                poly = project_box(cam, t[0], t[1], t[2], yaw0, m["shape"])
                if poly is not None:
                    poly = [(min(max(x, 0.0), W), min(max(y, 0.0), H))
                            for x, y in poly]
                if poly is None:
                    n["badshape"] += 1
                    new_lines.append(" ".join(old))
                    continue
            norm = norm_poly(poly)
            new_lines.append(f"{meta['cls']} "
                             + " ".join(f"{v:.5f}" for v in norm))
        if args.write:
            try:
                with open(lp, "w") as fh:
                    fh.write("\n".join(new_lines) + "\n")
            except PermissionError:
                # root-owned file from a still-running capture container —
                # post-fix frames carry correct labels anyway; skip
                n["perm"] = n.get("perm", 0) + 1
    print(f"labels: fit(verified)={n['fit']} approx(analytic-fallback)="
          f"{n['approx']} direct(post-fix kept)={n['direct']} "
          f"neg-recs={n['neg']} nocam={n['nocam']} badshape={n['badshape']} "
          f"perm-skipped={n.get('perm', 0)}")
    for ex in approx_examples:
        print("  approx:", ex)
    if not args.write:
        print("dry run — pass --write to rewrite label files")


if __name__ == "__main__":
    main()
