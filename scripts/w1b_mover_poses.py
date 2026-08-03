#!/usr/bin/env python3
"""w1b_mover_poses — sample /world/<world>/dynamic_pose/info twice, DT seconds
apart, and print per-mover pose deltas + implied speeds (W1b validation:
every mover's pose changes plausibly — walkers slow, cars on route). Also
prints the full model list so missing movers are loud.

  docker exec w1b-demo bash -lc 'cd /workspace && uv run --no-project python \
      scripts/w1b_mover_poses.py --dt 10'
"""
import argparse
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV

MOVERS = ["car_1", "car_2", "car_3", "walker_1", "walker_2"]


def sample(node, world, out):
    def cb(msg):
        for p in msg.pose:
            out[p.name] = (p.position.x, p.position.y, p.position.z)
    sub = node.subscribe(GzPoseV, f"/world/{world}/dynamic_pose/info", cb)
    assert sub, "subscribe failed"
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end and len(out) < len(MOVERS):
        time.sleep(0.1)
    return sub


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dt", type=float, default=10.0)
    ap.add_argument("--world", default="demo")
    args = ap.parse_args()

    node = GzNode()
    a, b = {}, {}
    sub_a = sample(node, args.world, a)
    snap_a = dict(a)            # frozen: the subscription keeps refreshing `a`
    time.sleep(args.dt)
    sub_b = sample(node, args.world, b)
    _keep_alive = (sub_a, sub_b)   # gz kills subscriptions with the handles
    print("models in dynamic_pose:", sorted(b))
    rc = 0
    for name in MOVERS:
        if name not in snap_a or name not in b:
            print(f"{name}: MISSING from dynamic_pose/info")
            rc = 1
            continue
        pa, pb = snap_a[name], b[name]
        d = sum((u - v) ** 2 for u, v in zip(pa, pb)) ** 0.5
        print(f"{name}: t0=({pa[0]:.2f},{pa[1]:.2f},{pa[2]:.2f}) "
              f"t1=({pb[0]:.2f},{pb[1]:.2f},{pb[2]:.2f}) "
              f"delta={d:.2f} m over {args.dt:.0f}s = {d / args.dt:.2f} m/s")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
