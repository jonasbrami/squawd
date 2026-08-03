"""W4 orbitwatch (in-container): the dedicated orbit instrument.

10 Hz synchronized log of the orbit's ground truth + perception view:
drone E/N/alt/speed/heading (PX4 vehicle_local_position), the designated
target's EKF e/n + health (/pilot/detections), and the derived horizontal
radius. The radius/time series + smoothness stats are computed from this
log by w4_orbit_verdict.py (host side).

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w4_orbit/w4_orbitwatch.py [seconds]
"""
import json
import math
import time
import sys

from agents.core.bus import STATE_QOS, RosBridge


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleLocalPosition
    bridge = RosBridge(node_name="w4_orbitwatch")
    bridge.subscribe("/pilot/detections", String, STATE_QOS)
    bridge.subscribe("/px4_0/fmu/out/vehicle_local_position",
                     VehicleLocalPosition)
    bridge.start()
    print(f"BEGIN wall {time.time():.3f}", flush=True)
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < seconds:
        msg = bridge.latest("/pilot/detections")
        lp = bridge.latest("/px4_0/fmu/out/vehicle_local_position")
        if msg is None or lp is None or msg is last:
            time.sleep(0.03)
            continue
        last = msg
        snap = json.loads(msg.data)
        tr = snap.get("track") or {}
        tgt_name = tr.get("target")
        tgt = None
        for c in snap.get("contacts") or []:
            if c.get("name") == tgt_name:
                tgt = c
                break
        t = time.monotonic() - t0
        row = {"t": round(t, 2),
               "mode": tr.get("state"), "tgt": tgt_name,
               "de": round(float(lp.y), 2), "dn": round(float(lp.x), 2),
               "alt": round(-float(lp.z), 2),
               "spd": round(math.hypot(float(lp.vx), float(lp.vy)), 2),
               "hdg": round(math.degrees(float(lp.heading)), 1),
               "gap": tr.get("gap_m")}
        if tgt is not None and tgt.get("e") is not None:
            te, tn = float(tgt["e"]), float(tgt["n"])
            row["te"], row["tn"] = round(te, 2), round(tn, 2)
            row["radius"] = round(math.hypot(row["de"] - te, row["dn"] - tn), 2)
            row["health"] = tgt.get("health")
        print(json.dumps(row), flush=True)


if __name__ == "__main__":
    main()
