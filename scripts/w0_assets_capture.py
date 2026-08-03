#!/usr/bin/env python3
"""w0_assets_capture — save N PNG frames per static camera of the `assets`
world (W0.1 detector-on-rendered-assets gate). Standalone gz-transport
subscriber: same topic shape and Image-message handling as
agents/core/camera.py:GzCameras, but deliberately shares no production code.

  docker exec w0-assets bash -lc 'cd /workspace && uv run --no-project python \
      scripts/w0_assets_capture.py --out evals/out/w0_detector_assets/frames --n 10'
"""
import argparse
import os
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image as GzImage
from PIL import Image as PILImage

CAMS = ["cam10_low", "cam25_low", "cam40_low",
        "cam10_high", "cam25_high", "cam40_high", "cam_house"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="evals/out/w0_detector_assets/frames")
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--world", default=os.environ.get("GZ_WORLD", "assets"))
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    node = GzNode()
    counts = {c: 0 for c in CAMS}
    cbs = []                                # keep callbacks alive for gz
    for cam in CAMS:
        topic = (f"/world/{args.world}/model/{cam}/link/link"
                 f"/sensor/IMX214/image")

        def make_cb(cam: str):
            def cb(msg):
                if counts[cam] >= args.n:
                    return
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
        if all(counts[c] >= args.n for c in CAMS):
            break
        time.sleep(0.2)
    print("captured:", counts, flush=True)
    missing = [c for c in CAMS if counts[c] == 0]
    if missing:
        print("NO FRAMES from:", missing, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
