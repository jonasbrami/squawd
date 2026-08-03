"""W3 post-lock autopsy probe (in-container): 10 Hz watch of the perception
snapshot for N seconds after a lock, tracing the designated contact's
life — poses membership, health, rebirth ids — and the track/beam views,
to pin down why the track op exits early.

  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/w3_integration/w3_autopsy.py <locked_name> [seconds]
"""
import json
import sys
import time

from agents.core.bus import STATE_QOS, RosBridge


def main() -> None:
    target = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w3_autopsy")
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
        cs = snap.get("contacts") or []
        have = target in [c["name"] for c in cs]
        brief = [(c["name"], c.get("health"), c.get("position_src"),
                  None if c.get("range_m") is None else round(c["range_m"], 1),
                  None if c.get("age_s") is None else round(c["age_s"], 1))
                 for c in cs]
        tr, bm = snap.get("track") or {}, snap.get("beam") or {}
        print(f"t={time.monotonic() - t0:5.1f} have={have} "
              f"track={tr.get('state')}/{tr.get('target')} "
              f"beam={bm.get('status')} "
              f"dets={len(snap.get('dets') or [])} contacts={brief}",
              flush=True)


if __name__ == "__main__":
    main()
