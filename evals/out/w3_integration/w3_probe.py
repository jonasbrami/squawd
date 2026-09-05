"""W3 detection-recall probe (in-container): dump every /pilot/detections
cycle for N seconds — dets (cls/conf/box) + confirmed contacts — so the
engagement geometry is chosen on measured recall, not guesswork.

  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/w3_integration/w3_probe.py [seconds]
"""
import json
import sys
import time

from agents.core.bus import STATE_QOS, RosBridge


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    from std_msgs.msg import String
    bridge = RosBridge(node_name="w3_probe")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    bridge.start()
    t0 = time.monotonic()
    last = None
    n_cycles = n_car_dets = 0
    while time.monotonic() - t0 < seconds:
        msg = bridge.latest("/pilot/detections")
        if msg is None or msg is last:
            time.sleep(0.05)
            continue
        last = msg
        snap = json.loads(msg.data)
        n_cycles += 1
        dets = snap.get("dets") or []
        cars = [d for d in dets if d.get("cls") in ("car", "truck", "bus")]
        n_car_dets += bool(cars)
        line = (f"t={time.monotonic() - t0:5.1f} dets={len(dets)} "
                f"cars={[(round(d['conf'], 2), [round(v) for v in d['xyxy']])
                        for d in cars]} "
                f"contacts={[(c['name'], c.get('health'),
                              None if c.get('range_m') is None
                              else round(c['range_m'], 1))
                             for c in snap.get('contacts') or []]}")
        print(line, flush=True)
    print(f"PROBE: {n_cycles} cycles, car det present in {n_car_dets} "
          f"({100.0 * n_car_dets / max(n_cycles, 1):.0f}%)", flush=True)


if __name__ == "__main__":
    main()
