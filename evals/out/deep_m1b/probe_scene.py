"""M1b probe (in-container): drone pose, live mover poses, building boxes —
to pick yaw headings that frame a house + a car for the acceptance frames."""
import math
import os
import time

from agents.core.bus import RosBridge
from agents.core.gzposes import GzPoses
from agents.perception.perception import yaw_deg_to
from agents.world import World

MOVERS = ["car_1", "car_2", "car_3", "walker_1", "walker_2"]

bridge = RosBridge(node_name="m1b_probe")
from px4_msgs.msg import VehicleLocalPosition
bridge.subscribe("/px4_0/fmu/out/vehicle_local_position", VehicleLocalPosition)
world = World()
gz = GzPoses(os.environ.get("GZ_WORLD", "demo"), MOVERS)
bridge.start()
gz.anchor()
time.sleep(2.0)
st = world.drone_state(bridge, 0)
print("drone_state:", st)
e0, n0 = st[0], st[1]
for name, p in sorted(gz.poses().items()):
    d = math.hypot(p[0] - e0, p[1] - n0)
    print(f"{name}: E{p[0]:.1f} N{p[1]:.1f} dist {d:.0f} m "
          f"yaw {yaw_deg_to(e0, n0, p[0], p[1]):.0f}")
for b in world.buildings():
    cx = b.get("cx", b.get("e"))
    cy = b.get("cy", b.get("n"))
    d = math.hypot(cx - e0, cy - n0)
    print(f"{b['name']}: E{cx:.1f} N{cy:.1f} dims={b.get('dims')} "
          f"dist {d:.0f} m yaw {yaw_deg_to(e0, n0, cx, cy):.0f}")
