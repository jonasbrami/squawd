"""FlightOps: the drone's flight primitives as plain async methods.

One instance per drone, over its MAVSDK `System` + the shared `World`/`RosBridge`.
Each method performs the maneuver and returns a short status string; it raises on
bad input or SDK failure (the tool layer turns that into an error result). No
Claude-Agent-SDK coupling lives here, so the flight logic reads on its own and is
reusable outside the swarm.

Frames: world is ENU (east/north/up); MAVSDK goto_location takes lat/lon/AMSL +
yaw (deg, 0=N, +clockwise). World points are converted to GPS via the drone's
live fix (see _world_to_geo).
"""
import asyncio
import math

from mavsdk.action import OrbitYawBehavior

from agents.core.geo import GeoPoint, offset_point
from agents import perception

COMPASS = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0, "east": 90.0, "e": 90.0,
           "southeast": 135.0, "se": 135.0, "south": 180.0, "s": 180.0, "southwest": 225.0,
           "sw": 225.0, "west": 270.0, "w": 270.0, "northwest": 315.0, "nw": 315.0}


class FlightOps:
    def __init__(self, drone, world, bridge, i: int, n: int) -> None:
        self.drone = drone
        self.world = world
        self.bridge = bridge
        self.i = i
        self.n = n
        self.name = f"drone_{i}"

    # ---- helpers ----
    def _alt(self):
        p = self.bridge.latest(f"/px4_{self.i}/fmu/out/vehicle_local_position")
        return None if p is None else -p.z

    def _keep_yaw(self) -> float:
        st = self.world.drone_state(self.bridge, self.i)
        return math.degrees(st[3]) if st else 0.0

    async def _world_to_geo(self, t_e, t_n, t_u) -> GeoPoint:
        """Convert a world ENU point to a GeoPoint, relative to the drone's live GPS fix."""
        me = self.world.world_xy(self.bridge, self.i)
        pos = await anext(self.drone.telemetry.position())
        origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        if me is None:
            return origin
        return offset_point(origin, t_n - me[1], t_e - me[0], t_u - me[2])

    def _resolve_xy(self, target, east=None, north=None):
        """(east, north) for a symbolic target name, explicit east/north, or None."""
        if target:
            return self.world.resolve_xy(target, self.bridge, self.n)
        if east is not None or north is not None:
            me = self.world.world_xy(self.bridge, self.i)
            return (float(east if east is not None else (me[0] if me else 0.0)),
                    float(north if north is not None else (me[1] if me else 0.0)))
        return None

    # ---- primitives ----
    async def take_off(self, altitude=10.0) -> str:
        target = float(altitude)
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(target)
        await self.drone.action.takeoff()
        for _ in range(20):                      # gate on reaching altitude (safety)
            await asyncio.sleep(1)
            a = self._alt()
            if a is not None and a >= target * 0.9:
                break
        a = self._alt()
        return f"{self.name} airborne at {a:.0f}m" if a else f"{self.name} airborne"

    async def fly(self, north=0.0, east=0.0, up=0.0) -> str:
        north, east, up = float(north), float(east), float(up)
        pos = await anext(self.drone.telemetry.position())
        origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        tgt = offset_point(origin, north, east, up)
        yaw = math.degrees(math.atan2(east, north)) if (north or east) else self._keep_yaw()
        await self.drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                              tgt.absolute_altitude_m, yaw)
        return f"{self.name} moving N{north:+.0f} E{east:+.0f} U{up:+.0f}"

    async def goto(self, target="", east=None, north=None, up=None, heading="travel") -> str:
        me = self.world.world_xy(self.bridge, self.i)
        target = str(target or "").strip().lower()
        xy = self._resolve_xy(target, east, north)
        if xy is None:
            raise ValueError("need a target or east/north")
        t_e, t_n = xy
        t_u = float(up) if up is not None else (me[2] if me else 10.0)
        tgt = await self._world_to_geo(t_e, t_n, t_u)
        hh = str(heading or "travel").strip().lower()
        if hh in COMPASS:
            yaw = COMPASS[hh]
        elif hh not in ("", "travel") and hh.lstrip("-").replace(".", "", 1).isdigit():
            yaw = float(hh)
        elif me and (abs(t_e - me[0]) > 0.5 or abs(t_n - me[1]) > 0.5):
            yaw = math.degrees(math.atan2(t_e - me[0], t_n - me[1]))   # face travel
        else:
            yaw = self._keep_yaw()
        await self.drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                              tgt.absolute_altitude_m, yaw)
        return f"{self.name} -> E{t_e:.0f} N{t_n:.0f} alt {t_u:.0f}"

    async def orbit(self, target="", east=None, north=None, radius=12.0, speed=3.0,
                    direction="cw", alt=None) -> str:
        target = str(target or "").strip().lower()
        xy = self._resolve_xy(target, east, north)
        if xy is None:
            raise ValueError("can't resolve orbit center")
        me = self.world.world_xy(self.bridge, self.i)
        radius = abs(float(radius))
        speed = abs(float(speed))
        alt_v = float(alt) if alt is not None else (me[2] if me else 12.0)
        direction = str(direction or "cw").strip().lower()
        # MAVLink DO_ORBIT: direction is the SIGN of radius (+ = clockwise, - = ccw).
        signed = -radius if direction in ("ccw", "counterclockwise", "anticlockwise") else radius
        center = await self._world_to_geo(xy[0], xy[1], alt_v)
        await self.drone.action.do_orbit(signed, speed, OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
                                         center.latitude_deg, center.longitude_deg,
                                         center.absolute_altitude_m)
        return (f"{self.name} orbiting {target or 'point'} r={radius:.0f}m {direction} "
                f"at {alt_v:.0f}m, camera on center")

    async def hover(self) -> str:
        await self.drone.action.hold()
        return f"{self.name} holding"

    async def set_speed(self, speed=5.0) -> str:
        v = abs(float(speed))
        await self.drone.action.set_current_speed(v)
        return f"{self.name} speed {v:.1f} m/s"

    async def face(self, target="") -> str:
        tgt = str(target or "").strip().lower()
        if tgt in COMPASS:
            yaw = COMPASS[tgt]
        else:
            me = self.world.world_xy(self.bridge, self.i)
            txy = self.world.resolve_xy(tgt, self.bridge, self.n)
            if me is None or txy is None:
                raise ValueError(f"can't resolve target '{tgt}'")
            yaw = perception.yaw_deg_to(me[0], me[1], txy[0], txy[1])
        pos = await anext(self.drone.telemetry.position())
        await self.drone.action.goto_location(pos.latitude_deg, pos.longitude_deg,
                                              pos.absolute_altitude_m, yaw)
        return f"{self.name} turning to face {tgt} (heading {yaw:.0f}deg)"

    async def land(self) -> str:
        await self.drone.action.land()
        for _ in range(30):                      # confirm touchdown (safety)
            await asyncio.sleep(1)
            a = self._alt()
            if a is not None and a < 0.5:
                break
        return f"{self.name} landed"

    def scan(self) -> str:
        return perception.scan_text(self.world, self.bridge, self.i, self.n)
