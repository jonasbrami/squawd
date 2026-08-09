"""M3 staging probe (in-container): ONE direct sidecar detect with the
slowlane vocabulary on the current frame, bypassing the tap proxy (keeps
tap.log clean for the A/B windows). Prints the dets so the staging loop can
pick a heading with houses in view.

  PYTHONPATH=/workspace uv run --no-project python evals/out/deep_m3/probe.py
"""
import json
import os
import sys

from agents.core.camera import GzCameras
from agents.perception.deep_client import DeepClient

VOCAB = os.environ.get("DEEP_SLOWLANE_VOCAB",
                       "building,house,tree,pole,tower").split(",")


def main() -> None:
    cameras = GzCameras(1)
    frame = None
    for _ in range(40):                      # ≤4 s for a first frame
        import time
        frame = cameras.snapshot(0)
        if frame is not None:
            break
        time.sleep(0.1)
    if frame is None:
        sys.exit("no camera frame")
    client = DeepClient(base_url="http://host.docker.internal:8100",
                        token=os.environ.get("DEEP_TOKEN", ""))
    res = client.detect(frame, VOCAB, conf=0.05)
    if not res.ok:
        sys.exit(f"{res.status}: {res.detail}")
    print(json.dumps({"frame_seq": frame.seq, "sim_stamp": frame.sim_stamp,
                      "latency_ms": res.data["latency_ms"],
                      "dets": res.data["dets"]}, indent=1))


if __name__ == "__main__":
    main()
