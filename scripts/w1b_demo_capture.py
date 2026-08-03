#!/usr/bin/env python3
"""w1b_demo_capture — save N PNG frames per static evidence camera of the
`demo` world (W1b full-cast validation). Standalone gz-transport subscriber:
same topic shape and Image-message handling as agents/core/camera.py:GzCameras,
but deliberately shares no production code (mirrors w1a_demo_capture.py).

  docker exec w1b-demo bash -lc 'cd /workspace && uv run --no-project python \
      scripts/w1b_demo_capture.py --out evals/out/w1b_demo_world/frames --n 3'

--interval spaces saved frames per camera (e.g. 1.0 s to catch movers at
different phase points for the heading-alignment check).
"""
import argparse
import os
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

# mirror of sim/worlds/make_demo_world.py CAMERAS (standalone by design)
CAMS = ["cam_overview", "cam_loop", "cam_street", "cam_plaza", "cam_corner",
        "cam_walker", "cam_truck", "cam_house2"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="evals/out/w1b_demo_world/frames")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--cams", nargs="*", default=CAMS)
    ap.add_argument("--interval", type=float, default=0.0,
                    help="min seconds between saved frames per camera")
    ap.add_argument("--world", default=os.environ.get("GZ_WORLD", "demo"))
    ap.add_argument("--timeout", type=float, default=240.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    node = GzNode()
    counts = {c: 0 for c in args.cams}
    last = {c: 0.0 for c in args.cams}
    cbs = []                                # keep callbacks alive for gz
    for cam in args.cams:
        topic = (f"/world/{args.world}/model/{cam}/link/link"
                 f"/sensor/IMX214/image")

        def make_cb(cam: str):
            def cb(msg):
                if counts[cam] >= args.n:
                    return
                now = time.monotonic()
                if now - last[cam] < args.interval:
                    return
                last[cam] = now
                img = PILImage.frombytes("RGB", (msg.width, msg.height),
                                         bytes(msg.data))
                img.save(os.path.join(args.out, f"{cam}_{counts[cam]:02d}.png"))
                counts[cam] += 1
            return cb

        cb = make_cb(cam)
        cbs.append(cb)
        node.subscribe(GzImage, topic, cb)
        print("subscribed", topic, flush=True)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        if all(counts[c] >= args.n for c in args.cams):
            break
        time.sleep(0.2)
    print("captured:", counts, flush=True)
    missing = [c for c in args.cams if counts[c] == 0]
    if missing:
        print("NO FRAMES from:", missing, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
