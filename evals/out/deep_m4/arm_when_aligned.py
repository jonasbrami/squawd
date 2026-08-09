"""M4 reactive arming (in-container): the fresh stack's EKF yaw alignment
FLAPS with the timesync churn (estimator_status_flags.cs_yaw_align toggles;
health_and_arming_checks spams Preflight Fail: Yaw estimate error). Blind
takeoff attempts keep landing in the bad phases. This watcher subscribes
/px4_0/fmu/out/estimator_status_flags (via the repo RosBridge — PX4_QOS is
best-effort) and fires takeoff the moment cs_yaw_align (and gps) hold TRUE
for N consecutive 1 Hz samples, then verifies the climb.

  PYTHONPATH=/workspace uv run --no-project python \
      evals/out/deep_m4/arm_when_aligned.py [need_samples] [timeout_s]
"""
import asyncio
import sys
import threading
import time

from mavsdk import System
from px4_msgs.msg import EstimatorStatusFlags

from agents.core.bus import RosBridge


class Flags:
    def __init__(self):
        self.lock = threading.Lock()
        self.ok_run = 0
        self.last = None

    def cb(self, m):
        good = bool(m.cs_yaw_align) and bool(m.cs_gps) \
            and not m.cs_mag_field_disturbed
        with self.lock:
            self.ok_run = self.ok_run + 1 if good else 0
            self.last = (bool(m.cs_yaw_align), bool(m.cs_gps),
                         m.control_status_changes)

    def run(self):
        with self.lock:
            return self.ok_run


async def main() -> None:
    need = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 420.0
    flags = Flags()
    bridge = RosBridge(node_name="m4_arm_when_aligned")
    bridge.subscribe("/px4_0/fmu/out/estimator_status_flags",
                     EstimatorStatusFlags, callback=flags.cb)
    bridge.start()

    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for s in drone.core.connection_state():
        if s.is_connected:
            break

    t0 = time.monotonic()
    attempt = 0
    while time.monotonic() - t0 < timeout:
        if flags.run() >= need:
            attempt += 1
            print(f"[{time.monotonic()-t0:6.1f}s] aligned x{flags.run()} — "
                  f"takeoff attempt {attempt}", flush=True)
            try:
                await drone.action.set_takeoff_altitude(9.0)
                await drone.action.takeoff()
            except Exception as e:
                print(f"  rejected: {type(e).__name__} {e}", flush=True)
                await asyncio.sleep(5.0)
                continue
            tc = time.monotonic()
            while time.monotonic() - tc < 45.0:
                await asyncio.sleep(1.0)
                pos = await anext(drone.telemetry.position())
                if pos.relative_altitude_m > 6.0:
                    print(f"AIRBORNE at {pos.relative_altitude_m:.1f} m "
                          f"(attempt {attempt})", flush=True)
                    bridge.shutdown()
                    return
            print(f"  no climb (alt {pos.relative_altitude_m:.1f})", flush=True)
        await asyncio.sleep(0.3)
    print("TIMEOUT — never caught a stable alignment window", flush=True)
    bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
