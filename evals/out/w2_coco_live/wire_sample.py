"""Grab the FIRST /pilot/detections message whose dets carry masks and dump
it verbatim (W2 wire-format evidence). In-container, ROS env sourced:
  PYTHONPATH=/workspace uv run --no-project python evals/out/w2_coco_live/wire_sample.py <out>
"""
import sys
import time

from agents.core.bus import STATE_QOS, RosBridge


def main() -> None:
    out = sys.argv[1]
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w2_wire_sample")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    bridge.start()
    deadline = time.monotonic() + 240
    while time.monotonic() < deadline:
        msg = bridge.latest("/pilot/detections")
        if msg is not None and '"mask"' in msg.data:
            with open(out, "w") as f:
                f.write(msg.data)
            print("saved", out, len(msg.data), "bytes", flush=True)
            return
        time.sleep(0.5)
    raise SystemExit("no mask-bearing detection message in 240 s")


if __name__ == "__main__":
    main()
