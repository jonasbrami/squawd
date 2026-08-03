"""W3-run8 pre-flight diagnostic (in-container): read the pursuit-relevant
PX4 params via mavsdk (MPC_TILTMAX_AIR, MPC_XY_VEL_MAX) and report camera
liveness (gz camera frame seq advancing over 3 s). Evidence for the
attempt-1 view-twitch / camera-stall post-mortems.

  PYTHONPATH=/workspace:$PYTHONPATH uv run --no-project python \
      evals/out/w3_run8/w3_preflight.py
"""
import asyncio

from mavsdk import System

from agents.core.camera import GzCameras


async def main() -> None:
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    for name in ("MPC_TILTMAX_AIR", "MPC_XY_VEL_MAX"):
        try:
            v = await system.param.get_param_float(name)
            print(f"param {name} = {v}", flush=True)
        except Exception as e:
            print(f"param {name} read failed: {e}", flush=True)
    cams = GzCameras(1)
    f0 = cams.snapshot(0)
    await asyncio.sleep(3.0)
    f1 = cams.snapshot(0)
    s0 = None if f0 is None else f0.seq
    s1 = None if f1 is None else f1.seq
    adv = None if (s0 is None or s1 is None) else s1 - s0
    print(f"camera seq {s0} -> {s1} (advanced {adv} in 3 s)", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
