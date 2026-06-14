"""Prove MAVSDK + rclpy run together in one process.

Connects MAVSDK to PX4, subscribes to PX4's ROS2 vehicle_local_position via the
bridge, and for ~10s prints altitude from BOTH sources side by side.
"""
import asyncio

from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition

from agents.common.bus import RosBridge

TOPIC = "/fmu/out/vehicle_local_position"


async def main() -> None:
    bridge = RosBridge()
    bridge.subscribe(TOPIC, VehicleLocalPosition)
    bridge.start()

    drone = System(mavsdk_server_address="127.0.0.1", port=50051)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("MAVSDK connected to PX4")
            break

    for _ in range(10):
        ros_msg = bridge.latest(TOPIC)
        ros_alt = None if ros_msg is None else -ros_msg.z  # NED z -> altitude
        mav_pos = await anext(drone.telemetry.position())
        print(f"ROS2 alt={ros_alt!s:>8}  |  MAVSDK alt={mav_pos.relative_altitude_m:.2f} m")
        await asyncio.sleep(1)

    bridge.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
