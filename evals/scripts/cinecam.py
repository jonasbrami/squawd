"""Spawn/move the overhead cine-camera in a running gz world.

A static down-looking 720p camera model publishing on topic `cinecam`. Spawn
once per session, then re-aim per shot with `move` (set_pose works on static
models). Runs INSIDE the sim container.

usage: python3 cinecam.py spawn --world dynamic --x 50 --y 60 --z 230
       python3 cinecam.py move  --world dynamic --x -70 --y -70 --z 330
"""
import argparse

CAM_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="cinecam">
    <static>true</static>
    <pose>{x} {y} {z} 0 1.5708 1.5708</pose>
    <link name="link">
      <sensor name="cam" type="camera">
        <topic>cinecam</topic>
        <update_rate>20</update_rate>
        <camera>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>1280</width><height>720</height></image>
          <clip><near>0.1</near><far>1500</far></clip>
        </camera>
        <always_on>1</always_on>
      </sensor>
    </link>
  </model>
</sdf>"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["spawn", "move"])
    ap.add_argument("--world", default="dynamic")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, required=True)
    args = ap.parse_args()

    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_pb2 import Pose

    node = Node()
    if args.cmd == "spawn":
        req = EntityFactory()
        req.sdf = CAM_SDF.format(x=args.x, y=args.y, z=args.z)
        ok, resp = node.request(f"/world/{args.world}/create", req,
                                EntityFactory, Boolean, 5000)
    else:
        req = Pose()
        req.name = "cinecam"
        req.position.x, req.position.y, req.position.z = args.x, args.y, args.z
        # pitch 90 down then yaw 90: north-up/east-right in the image
        req.orientation.w, req.orientation.x = 0.5, -0.5
        req.orientation.y, req.orientation.z = 0.5, 0.5
        ok, resp = node.request(f"/world/{args.world}/set_pose", req,
                                Pose, Boolean, 5000)
    print(f"{args.cmd}: ok={ok} accepted={getattr(resp, 'data', None)}")


if __name__ == "__main__":
    main()
