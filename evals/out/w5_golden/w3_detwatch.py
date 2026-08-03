"""W3-run4 det-watch (in-container): 10 Hz dump of the raw /pilot/detections
det list (class, conf, box) + the track/beam view — distinguishes detector
recall starvation (no car det) from association rejection (car det present,
no contact update) during an engagement.

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w3_run4/w3_detwatch.py [seconds]
"""
import json
import sys
import time

from agents.core.bus import STATE_QOS, RosBridge


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w3_detwatch")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    bridge.start()
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < seconds:
        msg = bridge.latest("/pilot/detections")
        if msg is None or msg is last:
            time.sleep(0.04)
            continue
        last = msg
        snap = json.loads(msg.data)
        dets = snap.get("dets") or []
        brief = [(d.get("cls"), round(d.get("conf") or 0, 2),
                  [round(v) for v in d.get("xyxy") or []])
                 for d in dets]
        tr, bm = snap.get("track") or {}, snap.get("beam") or {}
        print(f"t={time.monotonic() - t0:6.1f} track={tr.get('state')}/"
              f"{tr.get('target')} beam={bm.get('status')} dets={brief}",
              flush=True)


if __name__ == "__main__":
    main()
