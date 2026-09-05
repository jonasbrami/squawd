"""M1b raw-frame capture (in-container, one shot per invocation).

Grabs the live gz frame via GzCameras (the exact RGB888 Frame contract of
agents/core/contact.py) and saves it UNTOUCHED as PNG + a meta sidecar
({seq, sim_stamp, w, h}) for the deep-perception M1b acceptance runs.

Run INSIDE pilot-sim:
  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/deep_m1b/capture_frames.py <tag>
"""
import json
import os
import sys
import time

from PIL import Image

from agents.core.camera import GzCameras

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    tag = sys.argv[1]
    cameras = GzCameras(1)
    deadline = time.monotonic() + 30
    f = None
    while time.monotonic() < deadline:
        f = cameras.snapshot(0)
        if f is not None:
            break
        time.sleep(0.2)
    if f is None:
        raise SystemExit("no camera frame within 30 s")
    Image.frombytes("RGB", (f.width, f.height), f.rgb).save(
        os.path.join(OUT_DIR, f"{tag}.png"))
    meta = {"seq": f.seq, "sim_stamp": f.sim_stamp, "w": f.width, "h": f.height}
    with open(os.path.join(OUT_DIR, f"{tag}.json"), "w") as fh:
        json.dump(meta, fh)
    print(f"{tag}: seq={f.seq} stamp={f.sim_stamp:.2f} {f.width}x{f.height}",
          flush=True)


if __name__ == "__main__":
    main()
