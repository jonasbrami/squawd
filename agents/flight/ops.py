"""FlightOps: the drone's flight primitives as plain async methods.

One instance per drone, over its MAVSDK `System` + the shared `World`/`RosBridge`.
Each method performs the maneuver and returns a short status string; it raises on
bad input or SDK failure (the tool layer turns that into an error result). No
Claude-Agent-SDK coupling lives here, so the flight logic reads on its own and is
reusable outside the pilot assembly.

Frames: world is ENU (east/north/up); MAVSDK goto_location takes lat/lon/AMSL +
yaw (deg, 0=N, +clockwise). World points are converted to GPS via the drone's
live fix (see _world_to_geo).
"""
import asyncio
import math
import textwrap
import traceback

from mavsdk.action import OrbitYawBehavior
from mavsdk.mission import MissionItem

from agents.core.geo import GeoPoint, offset_point
from agents import perception

COMPASS = {"north": 0.0, "n": 0.0, "northeast": 45.0, "ne": 45.0, "east": 90.0, "e": 90.0,
           "southeast": 135.0, "se": 135.0, "south": 180.0, "s": 180.0, "southwest": 225.0,
           "sw": 225.0, "west": 270.0, "w": 270.0, "northwest": 315.0, "nw": 315.0}


def _mission_item(**kw):
    """A MissionItem with every field defaulted (nan / enum NONE), overridable by
    its real SDK field name. Cuts the 14-required-arg boilerplate; hides nothing."""
    nan = float("nan")
    fields = dict(
        latitude_deg=nan, longitude_deg=nan, relative_altitude_m=nan,
        speed_m_s=nan, is_fly_through=True,
        gimbal_pitch_deg=nan, gimbal_yaw_deg=nan,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=nan, camera_photo_interval_s=nan,
        acceptance_radius_m=nan, yaw_deg=nan, camera_photo_distance_m=nan,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )
    fields.update(kw)
    return MissionItem(**fields)


DEFAULT_MISSION_TIMEOUT_S = 180.0

ARRIVE_TOL_M = 4.0          # horizontal arrival radius (well inside oracle tolerances)
ARRIVE_ALT_TOL_M = 2.5
ARRIVE_POLL_S = 0.5
ARRIVE_MIN_TIMEOUT_S = 15.0
ARRIVE_MARGIN = 2.5         # arrival timeout = MARGIN * dist / speed (accel/decel headroom)

# W3 codex R4 layer-5 pre-emption (the demo/hold_altitude coast only): coast
# yaw steers on the EKF-PREDICTED contact bearing, but never further out than
# this horizon from the last measurement — a constant-velocity ghost is
# fiction through the mover's 90deg corners, and any search behavior stays
# bounded to the same horizon.
COAST_PREDICT_HORIZON_S = 2.0


def _result_text(logs, body):
    """Prefix any log() lines before the result/traceback body."""
    if logs:
        return "logs:\n" + "\n".join(logs) + "\n\n" + body
    return body


class FlightOps:
    def __init__(self, drone, world, bridge, contacts=None,
                 envelope=None) -> None:
        self.drone = drone
        self.world = world
        self.bridge = bridge
        # O1: `contacts` is any mover-contact provider (poses/sim_time and
        # friends — VisionContacts in the pilot, GzPoses in eval tooling);
        self.contacts = contacts
        self.envelope = envelope
        self.name = "drone_0"
        self._speed = 5.0            # last commanded cruise speed (PX4 default)

    def _alt(self):
        p = self.bridge.latest(f"/px4_{0}/fmu/out/vehicle_local_position")
        return None if p is None else -p.z

    def _keep_yaw(self) -> float:
        st = self.world.drone_state(self.bridge, 0)
        return math.degrees(st[3]) if st else 0.0

    async def _world_to_geo(self, east, north, up) -> GeoPoint:
        """Convert a world ENU point to a GeoPoint, relative to the drone's live GPS fix.
        Params are named east/north/up so authored missions can call it by keyword."""
        me = self.world.world_xy(self.bridge, 0)
        pos = await anext(self.drone.telemetry.position())
        origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        if me is None:
            return origin
        return offset_point(origin, north - me[1], east - me[0], up - me[2])

    async def _await_arrival(self, t_e, t_n, t_u=None) -> str:
        """Poll the world-frame fix until within ARRIVE_TOL_M of (t_e, t_n) (and
        ARRIVE_ALT_TOL_M of t_u when given), or a distance-scaled timeout. Returns a
        status suffix; NEVER raises — a timeout reads 'still enroute' so the caller
        can wait again or re-command rather than see a hard error."""
        me = self.world.world_xy(self.bridge, 0)
        dist = math.hypot(t_e - me[0], t_n - me[1]) if me else 100.0
        t_max = max(ARRIVE_MIN_TIMEOUT_S, ARRIVE_MARGIN * dist / max(self._speed, 0.5))
        for _ in range(max(1, int(t_max / ARRIVE_POLL_S))):
            await asyncio.sleep(ARRIVE_POLL_S)
            me = self.world.world_xy(self.bridge, 0)
            if me is None:
                continue
            d = math.hypot(t_e - me[0], t_n - me[1])
            if d <= ARRIVE_TOL_M and (t_u is None or abs(t_u - me[2]) <= ARRIVE_ALT_TOL_M):
                return f"arrived ({d:.1f}m off target)"
        pos = f", now E{me[0]:.0f} N{me[1]:.0f}" if me else ""
        return f"STILL ENROUTE after {t_max:.0f}s{pos} — wait or re-issue"

    def _resolve_xy(self, target, east=None, north=None):
        """(east, north) for a symbolic target name, explicit east/north, or None."""
        if target:
            return self.world.resolve_xy(target)
        if east is not None or north is not None:
            me = self.world.world_xy(self.bridge, 0)
            return (float(east if east is not None else (me[0] if me else 0.0)),
                    float(north if north is not None else (me[1] if me else 0.0)))
        return None

    def _readopt_contact(self, name: str, last, vel=(0.0, 0.0), dt_s=0.0):
        """Name-churn adoption (M3a): when the named contact drops but a
        positioned same-class contact exists near the last estimate PREDICTED
        FORWARD (the frozen-position gate dies after gate_m/v = 1.4 s on this
        mover, well inside the lost window — fable-Q3), adopt it — the pursuit
        follows the OBJECT across the EKF's ephemeral-id rebirths. Ambiguity
        (two candidates in gate) adopts neither (hold/reacquire, not a wrong
        identity). Returns (new_name, pos) or None.

        W3 codex §3 (COCO profile ONLY — the provider's TrackerConfig carries
        a non-empty assoc_keys map; the mover/truth path runs the legacy law
        above BYTE-IDENTICAL): compare ASSOCIATION KEYS (a car->truck reclass
        keeps the object), require dt_s <= rebind_window_s, and search within
        min(8 m, gate_m + dt_s) of the CV-predicted point. With several
        matches adopt the NEAREST only when the rest form a <=2 m duplicate
        cluster around it (the observed double-birth of one physical car) or
        the runner-up is >=2 m farther — otherwise remain ambiguous."""
        if self.contacts is None or last is None:
            return None
        cls = name.split("_")[1] if "_" in name else None
        # the gate radius comes from the provider's own config when it has one
        # (the vision EKF knows its rebirth scale); 5 m default when it doesn't.
        prov = getattr(self.contacts, "cfg", None) or getattr(self.contacts, "config", None)
        gate = getattr(prov, "gate_m", 5.0)
        skeys = getattr(prov, "assoc_keys", None) or {}
        pe, pn = last[0] + vel[0] * dt_s, last[1] + vel[1] * dt_s
        if skeys and cls is not None:
            if dt_s > getattr(prov, "rebind_window_s", 2.0):
                return None                 # expired: a new object, not lineage
            key = skeys.get(cls, cls)
            radius = min(8.0, gate + dt_s)
            matches = []
            for cand, pos in self.contacts.poses().items():
                ccls = cand.split("_")[1] if "_" in cand else None
                if ccls is None or skeys.get(ccls, ccls) != key:
                    continue
                d = math.hypot(pos[0] - pe, pos[1] - pn)
                if d <= radius:
                    matches.append((d, cand, pos))
            if not matches:
                return None
            matches.sort(key=lambda m: m[0])
            (d0, best, p0), rest = matches[0], matches[1:]
            if rest:
                cluster = all(math.hypot(p[0] - p0[0], p[1] - p0[1]) <= 2.0
                              for _d, _c, p in rest)
                if not cluster and rest[0][0] - d0 < 2.0:
                    return None             # genuinely ambiguous: hold (LOST)
            return (best, p0)
        matches = []
        for cand, pos in self.contacts.poses().items():
            if cls is None or cand.startswith(f"vis_{cls}_") or cand == name:
                d = math.hypot(pos[0] - pe, pos[1] - pn)
                if d <= gate:
                    matches.append((d, cand, pos))
        if len(matches) != 1:
            return None
        return (matches[0][1], matches[0][2])
    async def _acquire(self, name: str, alt: float, budget_s: float = 45.0):
        """O6 acquisition (§3.10): designate the bearing-only contact, then
        yaw onto the measured bearing and bias altitude toward its elevation
        (co-altitude so the forward beam intersects the target), holding for
        the first beam lock with bounded patience. Returns the ACQUIRED
        altitude (carried into the pursuit — the baro/GPS-height drift keeps
        moving after lock) on RANGE_LOCKED/WORLD_TRACKED, None on budget
        exhaustion — never a silent ghost-chase."""
        import time as _t
        designate = getattr(self.contacts, "designate", None)
        if not callable(designate):
            return None
        try:
            designate(name)
        except Exception:
            return None
        obs_fn = getattr(self.contacts, "observation", None)
        ts_fn = getattr(self.contacts, "track_state", None)

        def _locked() -> bool:
            """Success means a ToF-LOCKED state — never a bare pose (a geom-
            born or predicted pose is not an acquisition, codex-R3)."""
            if not callable(ts_fn):
                return name in self.contacts.poses()
            return ts_fn(name) in ("RANGE_LOCKED", "WORLD_TRACKED")

        t0 = _t.monotonic()
        try:
            from mavsdk.offboard import PositionNedYaw, VelocityNedYaw
            me0 = self.world.drone_state(self.bridge, 0)
            lp = self.bridge.latest(f"/px4_{0}/fmu/out/vehicle_local_position")
            off_n, off_e, off_d = lp.x - me0[1], lp.y - me0[0], lp.z + me0[2]
            # pre-start stream MUST hold the CURRENT heading: a yaw=0.0 init
            # slams the nose to north when offboard engages and the box is
            # out of frame before the servo loop's first command (v9.1).
            yaw0 = math.degrees(me0[3]) % 360
            await self.drone.offboard.set_position_velocity_ned(
                PositionNedYaw(me0[1] + off_n, me0[0] + off_e, off_d - alt,
                               yaw0),
                VelocityNedYaw(0.0, 0.0, 0.0, yaw0))
            for _ in range(3):
                await asyncio.sleep(0.1)
                await self.drone.offboard.set_position_velocity_ned(
                    PositionNedYaw(me0[1] + off_n, me0[0] + off_e, off_d - alt,
                                   yaw0),
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw0))
            try:
                await self.drone.offboard.start()
            except Exception:
                pass
            ax_prev, ax_t, ax_rate = None, 0.0, 0.0
            last_seen = _t.monotonic()
            alt0 = alt            # proportional servo's reference (no windup)
            while _t.monotonic() - t0 < budget_s:
                if _locked():
                    return alt
                obs = obs_fn(name) if callable(obs_fn) else None
                me = self.world.drone_state(self.bridge, 0)
                if me is None:
                    await asyncio.sleep(0.5)
                    continue
                b = getattr(obs, "bearing_deg", None)
                # the DTO field is elevation_deg (codex-R3: the servo read
                # `elev_deg` — a nonexistent field — and never fired live)
                e = getattr(obs, "elevation_deg", None)
                # offboard BEARING-HOMING (M3b): image-servo the yaw to null the
                # det's pixel angle (immune to the EKF yaw/declination offset
                # that broke heading-based aim) AND advance along the beam in
                # a SEGMENTED creep-and-listen (a bearing-only contact has no
                # range yet — the beam IS the acquisition): LISTEN first (the
                # beam may lock from the rendezvous standoff), then creep 3 s,
                # listen again; after N creep segments without a lock, HOLD —
                # a monotonic creep drives into the target (observed live:
                # min gap 0.6 m, LOST at t+5s, v8.1).
                # foot_px must be FRESH: it persists on the track through
                # COASTING, so an un-gated fp makes the blind branch
                # unreachable and aims off a ghost (codex-R3).
                fp_raw = getattr(obs, "foot_px", None)
                fp = (fp_raw if fp_raw is not None
                      and getattr(obs, "age_s", 99.0) < 0.5 else None)
                lp = self.bridge.latest(
                    f"/px4_{0}/fmu/out/vehicle_local_position")
                if lp is None:
                    # a telemetry hiccup must not crash the acquisition
                    # (fable-R3): both branches below dereference lp
                    await asyncio.sleep(0.2)
                    continue
                own_sp = math.hypot(getattr(lp, "vx", 0.0),
                                    getattr(lp, "vy", 0.0))
                ctx = getattr(self.contacts, "set_beam_context", None)
                if callable(ctx):
                    # the FLIGHT layer owns the envelope context, with the
                    # vehicle's measured speed (codex-R3: the pursuit fed the
                    # TARGET's speed and the eval collector raced it)
                    ctx(mode="shadow", own_speed_mps=own_sp)
                if fp is not None and me is not None:
                    last_seen = _t.monotonic()
                    from agents.perception.projection import (
                        pixel_to_angles, vfov_deg)
                    ax, _ay = pixel_to_angles(fp[0], fp[1], 640, 360)
                    # bearing-rate FEEDFORWARD (M3b v8.4): an orbiting box
                    # drags a proportional-only servo ~10 deg behind — past
                    # the envelope's off-boresight gate, so the one VALID
                    # beam sample gets declined. Lead the command with the
                    # EMA'd pixel-bearing rate (clamped) so the box HOLDS at
                    # boresight through the orbit.
                    now_t = _t.monotonic()
                    if ax_prev is not None and now_t > ax_t:
                        rate = (ax - ax_prev) / (now_t - ax_t)
                        ax_rate = 0.7 * ax_rate + 0.3 * rate
                    ax_prev, ax_t = ax, now_t
                    # gain 0.2 s / ±0.1 rad clamp: the lead feeds the yaw's
                    # own motion back into the rate estimate — a bigger
                    # clamp lets the loop ring the box around boresight
                    lead = max(-0.1, min(0.1, ax_rate * 0.2))
                    cmd_yaw = (math.degrees(me[3]) + math.degrees(ax)
                               + math.degrees(lead)) % 360
                    off_n, off_e, off_d = lp.x - me[1], lp.y - me[0], lp.z + me[2]
                    LISTEN_S, CREEP_S, MAX_CREEPS = 4.0, 3.0, 4
                    el = _t.monotonic() - t0
                    cycle = el % (LISTEN_S + CREEP_S)
                    creeps_done = int(el // (LISTEN_S + CREEP_S))
                    do_creep = cycle >= LISTEN_S and creeps_done < MAX_CREEPS
                    b_rad = math.radians(cmd_yaw)
                    vn, ve = (2.0 * math.cos(b_rad), 2.0 * math.sin(b_rad)) \
                        if do_creep else (0.0, 0.0)
                    await self.drone.offboard.set_position_velocity_ned(
                        PositionNedYaw(me[1] + off_n, me[0] + off_e,
                                       off_d - alt, cmd_yaw),
                        VelocityNedYaw(vn, ve, 0.0, cmd_yaw))
                elif b is not None:
                    # BLIND RECOVERY (M3b v8.5): the det left the frame — a
                    # frozen yaw deadlocks (the orbit carries the box away
                    # and the camera never re-sees it). Aim by the track's
                    # last bearing; sweep ±60° around it at 0.5 Hz — wide
                    # enough to catch a ~20 deg/s orbiter, fast enough to
                    # re-detect inside lost_s (2 s).
                    blind = _t.monotonic() - last_seen
                    sweep = (60.0 * math.sin(2.0 * math.pi * 0.5 * blind)
                             if blind > 0.3 else 0.0)
                    cmd_yaw = (float(b) + sweep) % 360
                    off_n, off_e, off_d = lp.x - me[1], lp.y - me[0], lp.z + me[2]
                    await self.drone.offboard.set_position_velocity_ned(
                        PositionNedYaw(me[1] + off_n, me[0] + off_e,
                                       off_d - alt, cmd_yaw),
                        VelocityNedYaw(0.0, 0.0, 0.0, cmd_yaw))
                if e is not None and fp is not None:
                    # co-altitude servo on the box's VERTICAL CENTRE — the
                    # footpoint/base is erosion-biased ~+1.7° (v11: the servo
                    # equilibrated ~1 m HIGH, beam over the box top); the
                    # sunlit top edge is clean, so centre = base + half the
                    # angular height is robust. Drive the centre to boresight
                    # (camera = box centre). PROPORTIONAL, no integrator
                    # (v10: the accumulator wound +2.5 m past the box).
                    xyxy = getattr(obs, "bbox_xyxy", None)
                    if xyxy is not None:
                        fy = (360.0 / 2.0) / math.tan(
                            math.radians(vfov_deg(640, 360)) / 2.0)
                        half_ang = math.degrees(
                            math.atan(((xyxy[3] - xyxy[1]) / 2.0) / fy))
                        e_c = float(e) + half_ang
                    else:
                        e_c = float(e) + 3.4    # footpoint fallback (~mid-box)
                    alt = max(0.5, alt0 + 0.35 * e_c)
                await asyncio.sleep(0.1)
        finally:
            try:
                await asyncio.shield(self.drone.offboard.stop())
            except Exception:
                try:
                    await self.drone.action.hold()
                except Exception:
                    pass
            clear = getattr(self.contacts, "clear_designation", None)
            if callable(clear) and name not in self.contacts.poses():
                try:
                    clear()
                except Exception:
                    pass
        return alt if _locked() else None


    async def take_off(self, altitude=10.0) -> str:
        target = float(altitude)
        await self._arm_robust()
        await self.drone.action.set_takeoff_altitude(target)
        await self.drone.action.takeoff()
        for _ in range(20):                      # gate on reaching altitude (safety)
            await asyncio.sleep(1)
            a = self._alt()
            if a is not None and a >= target * 0.9:
                break
        a = self._alt()
        return f"{self.name} airborne at {a:.0f}m" if a else f"{self.name} airborne"

    async def _arm_robust(self, attempts: int = 3) -> None:
        """Arm via Hold first. PX4's Land nav_state has mode_req_prevent_arming:
        after any land() the bare arm() is denied ("cannot takeoff in current
        mode") until the intention is switched away from Land. SITL preflight
        transients (GPS drift flaps) also deny single arm attempts, so retry a
        few times — never bypassing the checks, just re-asking."""
        from mavsdk.action import ActionError
        last = None
        for _ in range(attempts):
            try:
                await self.drone.action.hold()
                await self.drone.action.arm()
                return
            except ActionError as e:
                last = e
                await asyncio.sleep(1.5)
        raise last

    async def fly(self, north=0.0, east=0.0, up=0.0, wait=True) -> str:
        north, east, up = float(north), float(east), float(up)
        me = self.world.world_xy(self.bridge, 0)     # for the arrival gate
        pos = await anext(self.drone.telemetry.position())
        origin = GeoPoint(pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m)
        tgt = offset_point(origin, north, east, up)
        yaw = math.degrees(math.atan2(east, north)) if (north or east) else self._keep_yaw()
        await self.drone.action.goto_location(tgt.latitude_deg, tgt.longitude_deg,
                                              tgt.absolute_altitude_m, yaw)
        base = f"{self.name} moving N{north:+.0f} E{east:+.0f} U{up:+.0f}"
        if not wait or me is None:
            return base
        return f"{base}; {await self._await_arrival(me[0] + east, me[1] + north, me[2] + up)}"

    async def goto(self, target="", east=None, north=None, up=None, heading="travel",
                   wait=True) -> str:
        me = self.world.world_xy(self.bridge, 0)
        target = str(target or "").strip().lower()
        xy = self._resolve_xy(target, east, north)
        if xy is None:
            raise ValueError("need a target or east/north")
        t_e, t_n = xy
        t_u = float(up) if up is not None else (me[2] if me else 10.0)
        # Refuse a commanded collision: a target inside a building's footprint
        # below its roof wedges the drone against the wall (observed: 90s of
        # grinding on a tower facade). A legible error lets the model re-plan.
        for b in getattr(self.world, "buildings", None) or []:
            if (abs(t_e - b["x"]) <= b["w"] / 2 and abs(t_n - b["y"]) <= b["d"] / 2
                    and t_u < b["h"] + 3.0):
                raise ValueError(
                    f"target E{t_e:.0f} N{t_n:.0f} at {t_u:.0f}m is INSIDE {b['name']} "
                    f"(centre E{b['x']:.0f} N{b['y']:.0f}, {b['w']:.0f}x{b['d']:.0f}m, "
                    f"{b['h']:.0f}m tall) — pick a standoff point or fly above it")
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
        base = f"{self.name} -> E{t_e:.0f} N{t_n:.0f} alt {t_u:.0f}"
        if not wait:
            return base + " (moving; not waiting)"
        return f"{base}; {await self._await_arrival(t_e, t_n, t_u)}"

    async def orbit(self, target="", east=None, north=None, radius=12.0, speed=3.0,
                    direction="cw", alt=None) -> str:
        target = str(target or "").strip().lower()
        xy = self._resolve_xy(target, east, north)
        if xy is None:
            raise ValueError("can't resolve orbit center")
        me = self.world.world_xy(self.bridge, 0)
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

    async def hover(self, seconds=0.0) -> str:
        """Hold position; with `seconds`, keep holding that long before returning —
        the only way to satisfy 'hold over X for N seconds' now that moves block
        (the post-turn settle no longer burns budget accruing dwell by accident)."""
        await self.drone.action.hold()
        s = min(max(float(seconds or 0.0), 0.0), 120.0)
        if s > 0:
            await asyncio.sleep(s)
            return f"{self.name} held {s:g}s"
        return f"{self.name} holding"

    async def set_speed(self, speed=5.0) -> str:
        v = abs(float(speed))
        # DO_CHANGE_SPEED only applies to the CURRENT reposition — the next
        # goto_location silently reverts to MPC_XY_CRUISE (observed live: a
        # "12 m/s" multi-leg dash flew at ~5). Set the cruise param too so the
        # commanded speed survives across gotos; evals soft_reset restores it.
        try:
            await self.drone.param.set_param_float("MPC_XY_CRUISE", min(v, 12.0))
        except Exception:
            pass                                 # param plugin unavailable: best effort
        await self.drone.action.set_current_speed(v)
        self._speed = max(v, 0.5)               # scales the arrival timeout
        return f"{self.name} speed {v:.1f} m/s"

    async def tune_pursuit_params(self) -> None:
        """Vision-pursuit tuning (fable-Q4): MPC_TILTMAX_AIR is the ONLY hard
        cap on pitch excursions reachable by offboard setpoints (PX4's jerk
        limits bypass offboard entirely — issue #18033); 12° caps horizontal
        accel at ~2.1 m/s², inside the level camera's FOV budget. And
        MPC_XY_VEL_MAX is the true absolute cap on v_ff + P-term correction
        (MAVSDK's velocity feedforward is NOT a constraint). Best effort —
        param failures are tolerated (degraded pursuit, not a crash)."""
        for name, val in [("MPC_TILTMAX_AIR", 12.0), ("MPC_XY_VEL_MAX", 6.0)]:
            try:
                await self.drone.param.set_param_float(name, val)
            except Exception:
                pass

    async def track(self, target="", mode="shadow", alt=None, duration_s=60.0,
                    within_m=15.0, speed=12.0, standoff_east=0.0,
                    standoff_north=0.0, acquire_budget_s: float = 45.0,
                    radius_m: float = 15.0, rate_dps: float = 15.0,
                    range_m: float | None = None,
                    hold_altitude: bool = False) -> str:
        """Real-time pursuit of a gz mover: 10 Hz offboard streaming of
        position + velocity-feedforward setpoints (PX4's cascade is the PD
        law — see agents/flight/track.py). Blocks until duration_s (capped)
        elapses, or returns EARLY in intercept mode the moment the horizontal
        gap closes within within_m. alt=None (the tool default) holds the
        CURRENT altitude — the same rule as orbit's (M6 commitment safeguard:
        an omitted alt must never be a silent fixed-default climb).

        W3a locked-object ops: mode="orbit" circles the contact at radius_m,
        rate_dps (sign = direction) with tangential feedforward; mode="shadow"
        with range_m holds a RADIAL stand-off (the /pilot/cmd standoff op).
        Both floor at track.MIN_ORBIT_RADIUS_M (the 7 m keep-out + margin).

        hold_altitude=True (W3 codex §4, the /pilot/cmd operator layer only —
        NOT exposed on the LLM tool schema): shadow holds the COMMANDED
        altitude and skips the M3b beam-geometry altitude profile (its sag
        descended the COCO demo pursuit out of the car's detection
        envelope), and the R2 radial floor raises range_m/radius_m to
        projection.min_pursuit_range_m(alt) — the level camera's blind cone
        grows with the hold altitude (docs/benchmarks/w3-rerun.md). R3: the
        held shadow takes the DIRECT reference lane (the 1 m/s^2 shaper lost
        the mover's 90deg corners — docs/benchmarks/w3-run3.md), the
        implicit lock ring defaults to R_min+2, and a radial escape
        feedforward engages inside R_min+1. R8: an image-edge barrier grows
        the guard by up to 4 m as the designated contact's bbox bottom nears
        the frame floor (docs/benchmarks/w3-run7.md). Default False keeps
        the M3b law byte-identical."""
        import time as _time

        from mavsdk.offboard import (OffboardError, PositionNedYaw,
                                     VelocityNedYaw)

        from agents.flight import track as trk
        from agents.perception import projection as proj

        if self.contacts is None:
            raise ValueError("track needs a contact provider (no mover feed)")
        name = str(target or "").strip()
        if alt is None:
            me_now = self.world.world_xy(self.bridge, 0)
            alt = me_now[2] if me_now else 12.0
        poses = self.contacts.poses()
        if name not in poses:
            # O4/O6: a BEARING-ONLY contact (a detection the EKF tracks but no
            # ToF beam has ranged yet) is not an error — designate it and run
            # the acquisition servo (§3.10): yaw onto the bearing, bias
            # altitude toward its elevation, wait for the first beam lock.
            # Only a name never seen in ANY form is a hard error.
            views = getattr(self.contacts, "all_views", None)
            bearing_only = []
            if callable(views):
                try:
                    bearing_only = [v.name for v in views()
                                    if getattr(v, "position_src", None)
                                    in (None, "none")]
                except Exception:
                    bearing_only = []
            if name in bearing_only:
                acq_alt = await self._acquire(name, alt, budget_s=acquire_budget_s)
                if acq_alt is None:
                    return f"{self.name} could NOT acquire {name}: no beam lock within the acquisition budget — the contact stays bearing-only; reposition and retry"
                # the pursuit runs at the ACQUIRED altitude — the elevation
                # servo moved it while winning the lock (codex-R3).
                alt = acq_alt
                poses = self.contacts.poses()
                if name not in poses:
                    return f"{self.name} {name} beam-locked but has no position yet — acquire again or hold"
            # unknown entirely — list what IS visible (positioned contacts
            # plus the bearing-only ones) so the model retries a real name.
            else:
                known = (", ".join(sorted(poses) + [f"{b} (bearing only)"
                                                   for b in bearing_only])
                         or "none seen yet")
                raise ValueError(f"unknown moving contact {name!r} (visible: "
                                 f"{known})")
        # W3 integration fix: an ALREADY-positioned (geom) contact never
        # passed through _acquire, so nothing designated it — but _feed_tof
        # idles while `designated is None`, so the ToF beam never fused on
        # the /api/lock click path and the cockpit's track banner/beam chip
        # stayed IDLE for the whole pursuit (design §5: the click perception
        # path runs through VisionContacts.designate; the pursuit already
        # feeds set_beam_context every tick — dead context without this).
        # Idempotent (post-_acquire re-designate is a no-op); truth-fed
        # providers (GzPoses) have no designate and are untouched.
        designate = getattr(self.contacts, "designate", None)
        if callable(designate):
            try:
                designate(name)
            except Exception:
                pass
        mode = str(mode or "shadow").strip().lower()
        if mode not in ("shadow", "intercept", "orbit"):
            raise ValueError("mode must be 'shadow', 'intercept' or 'orbit'")
        # W3 integration fix: apply the pursuit tuning HERE — until now only
        # the eval harnesses called tune_pursuit_params, so the live pilot /
        # operator click path pursued with PX4's stock MPC_TILTMAX_AIR: the
        # first dash pitched the body-fixed camera past the ±21° vfov edge,
        # the car left the frame, and the EKF dropped the contact inside
        # lost_s (five consecutive LOST pursuits observed live 2026-08-01).
        # Best effort and idempotent (the method already tolerates param
        # failures; the evals' explicit calls stay harmless).
        await self.tune_pursuit_params()
        alt = float(alt)
        within = max(1.0, float(within_m))
        speed = min(abs(float(speed)), trk.MAX_SPEED_MPS) or trk.MAX_SPEED_MPS
        dur = min(max(float(duration_s), 1.0), trk.MAX_DURATION_S)
        so_e, so_n = float(standoff_east), float(standoff_north)
        # keep-out floor: no orbit radius / stand-off range inside the bubble
        radius_m = max(trk.MIN_ORBIT_RADIUS_M, float(radius_m))
        range_m = (None if range_m is None
                   else max(trk.MIN_ORBIT_RADIUS_M, float(range_m)))
        if hold_altitude:
            # W3 codex R2 geometry law (the operator/demo path only): the
            # level camera's frame floor (half-vfov - 3deg margin) sets a
            # radial floor that GROWS with the commanded hold altitude —
            # inside it the target drops out of frame and the pursuit
            # LOST-breaks in its own blind cone (docs/benchmarks/w3-rerun.md).
            # A shadow without an explicit range defaults to the floor + 2 m
            # of corner-transient reserve (W3 codex R3: R_min is a
            # steady-state law, the ring needs margin for the mover's
            # corners); an explicit standoff stays floored at the floor
            # itself. hold_altitude=False keeps the M0-M6 defaults
            # byte-identical.
            r_min = proj.min_pursuit_range_m(alt)
            r_guard = r_min + 1.0
            radius_m = max(radius_m, r_min)
            range_m = r_min + 2.0 if range_m is None else max(range_m, r_min)
        orb = trk.OrbitPhase(radius_m, rate_dps)

        # world ENU -> PX4 local NED: constant offset from one simultaneous read
        me = self.world.world_xy(self.bridge, 0)
        lp = self.bridge.latest(f"/px4_{0}/fmu/out/vehicle_local_position")
        if me is None or lp is None:
            raise ValueError("no position fix yet — take off first")
        off_n, off_e, off_d = lp.x - me[1], lp.y - me[0], lp.z + me[2]

        def _sp(ref_e, ref_n, ref_u, ff_ve, ff_vn, yaw):
            return (PositionNedYaw(ref_n + off_n, ref_e + off_e,
                                   off_d - ref_u, yaw),
                    VelocityNedYaw(ff_vn, ff_ve, 0.0, yaw))

        tp = poses[name]
        yaw = math.degrees(math.atan2(tp[0] - me[0], tp[1] - me[1]))
        pos, vel = _sp(me[0], me[1], alt, 0.0, 0.0, yaw)
        started = False
        last_err = None
        for _attempt in range(4):
            for _ in range(5):                   # prime the stream before start()
                await self.drone.offboard.set_position_velocity_ned(pos, vel)
                await asyncio.sleep(0.05)
            try:
                await self.drone.offboard.start()
                started = True
                break
            except OffboardError as e:
                # PX4 can refuse the first start(s) while the offboard
                # handshake settles — re-prime the stream and retry.
                last_err = e
                await asyncio.sleep(0.3)
        if not started:
            raise ValueError(f"offboard start refused: {last_err._result.result} — are you airborne?"
                             ) from last_err

        est = trk.TargetEstimator()
        log = trk.TrackLog(within)
        wall0 = _time.monotonic()
        hit = None
        lost_since = None
        lost_wall = None
        lost_txt = None
        tp0 = tp
        v0 = (0.0, 0.0)
        t_last_seen = sim_now0 = self.contacts.sim_time()
        _shp = (me[0], me[1], 0.0, 0.0)
        prov = getattr(self.contacts, "cfg", None) or getattr(self.contacts, "config", None)
        lost_s = getattr(prov, "lost_s", 2.0)
        try:
            while _time.monotonic() - wall0 < dur:
                tp = self.contacts.poses().get(name)
                me = self.world.world_xy(self.bridge, 0)
                sim_now = self.contacts.sim_time()
                if tp is not None:
                    t_last_seen = sim_now
                if tp is None:
                    # M3a name-churn: the EKF rebirths contacts under new
                    # ephemeral ids — before declaring a loss, try to adopt a
                    # same-class candidate near the predicted-forward last
                    # estimate (the OBJECT outlives its ids).
                    adopted = self._readopt_contact(
                        name, tp0, vel=v0, dt_s=sim_now - t_last_seen)
                    if adopted is not None:
                        name, tp = adopted
                        self._last_track_name = name
                        # the EKF rebirth changed the id — move the
                        # designation (and with it ToF fusion + the cockpit
                        # banner) onto the ADOPTED contact, or fusion dies
                        # on the first churn (W3 integration fix).
                        if callable(designate):
                            try:
                                designate(name)
                            except Exception:
                                pass
                    if tp is None:
                        # really gone: hold position (the stream NEVER stops
                        # mid-call) and give the provider lost_s to return.
                        if lost_since is None:
                            lost_since, lost_wall = sim_now, _time.monotonic()
                        if (sim_now - lost_since > lost_s
                                or _time.monotonic() - lost_wall > lost_s):
                            lost_txt = (f"LOST: {name} dropped from contacts for >"
                                        f"{lost_s:.1f}s at t+{_time.monotonic() - wall0:.0f}"
                                        f"s — holding (min gap "
                                        f"{log.min_gap:.1f}m)")
                            break
                        hold_yaw = yaw
                        pos, vel = _sp(me[0] if me else tp0[0],
                                       me[1] if me else tp0[1],
                                       alt, 0.0, 0.0, hold_yaw)
                        await self.drone.offboard.set_position_velocity_ned(pos, vel)
                        await asyncio.sleep(1.0 / trk.CTRL_HZ)
                        continue
                    lost_since = None
                lost_since = None
                tp0 = tp
                if me is None:
                    await asyncio.sleep(1.0 / trk.CTRL_HZ)
                    continue
                # O3 velocity dispatch: take the provider's filtered velocity
                # when it has one (no re-differentiation lag); GzPoses has
                # none ({}), so the EMA finite-difference stays the fallback.
                vels = getattr(self.contacts, "velocities", None)
                vmap = vels() if callable(vels) else {}
                if name in vmap:
                    ve, vn = vmap[name]
                    # sanity-clamp the feed: a wild filter velocity would pull
                    # the lead point kilometres away — 6 m/s caps every mover.
                    vv = math.hypot(ve, vn)
                    if vv > 6.0:
                        ve, vn = ve * 6.0 / vv, vn * 6.0 / vv
                    est.feed_direct(ve, vn)
                    v0 = (ve, vn)
                else:
                    est.update(sim_now, tp[0], tp[1])
                    v0 = (est.ve, est.vn)
                # COASTING: for a ToF-fed (co-altitude) contact, the position
                # ghosts during a fusion drought — HOLDING position lets the
                # real target walk away (v18-v21: lock, then the gap grows to
                # 27-75 m and fusion never resumes). Re-close toward the
                # measured bearing at a creep (the beam IS the acquisition —
                # restore the close geometry where it re-locks, the SM's
                # COASTING -> ACQUIRING leg). For geom contacts the proven
                # M3a behavior holds the CURRENT position and keeps the nose
                # on the measured bearing — the stream NEVER stops mid-call
                # (O2). R4 (the demo/hold_altitude path only): the nose
                # follows the EKF-PREDICTED bearing instead, below; the mover
                # default keeps the stale bearing byte-identical.
                # 2026-08-03 codex-R4 coast-latch fix: BOTH branches below
                # anchor on the CURRENT position `me`, never `_shp` — the
                # direct lane (hold_altitude) initializes _shp at engagement
                # start and never updates it, so coast ticks used to command
                # the stale START point and fly the drone back mid-pursuit
                # (the operator's "immediately deviates").
                health_fn = getattr(self.contacts, "health", None)
                health = health_fn(name) if callable(health_fn) else "MEASURED"
                if health == "COASTING":
                    obs_fn = getattr(self.contacts, "observation", None)
                    obs = obs_fn(name) if callable(obs_fn) else None
                    b = getattr(obs, "bearing_deg", None)
                    coast_yaw = b if b is not None else yaw
                    src = getattr(obs, "range_src", None) if obs else None
                    if src == "tof" and b is not None:
                        b_rad = math.radians(coast_yaw)
                        pos, vel = _sp(me[0], me[1], alt,
                                       2.0 * math.cos(b_rad),
                                       2.0 * math.sin(b_rad), coast_yaw)
                    else:
                        if hold_altitude and src != "tof" \
                                and getattr(obs, "e", None) is not None:
                            # W3 codex R4: the coast froze yaw on the STALE
                            # measured bearing while the EKF kept predicting
                            # the contact — steer on the PREDICTED position's
                            # bearing so the level camera follows the mover
                            # through the grace period. Position/velocity
                            # stay HELD at the shaped point and the
                            # prediction never feeds association — yaw is its
                            # only consumer. The prediction is walked back to
                            # the COAST_PREDICT_HORIZON_S cap (the layer-5
                            # pre-emption) before the bearing is taken.
                            age = max(0.0, float(getattr(obs, "age_s", 0.0)
                                                 or 0.0))
                            back = age - min(age, COAST_PREDICT_HORIZON_S)
                            pe = float(obs.e) \
                                - float(getattr(obs, "ve", 0.0) or 0.0) * back
                            pn = float(obs.n) \
                                - float(getattr(obs, "vn", 0.0) or 0.0) * back
                            coast_yaw = math.degrees(
                                math.atan2(pe - me[0], pn - me[1]))
                        pos, vel = _sp(me[0], me[1], alt, 0.0, 0.0,
                                       coast_yaw)
                    await self.drone.offboard.set_position_velocity_ned(pos, vel)
                    await asyncio.sleep(1.0 / trk.CTRL_HZ)
                    continue
                gap = math.hypot(tp[0] - me[0], tp[1] - me[1])
                log.sample(_time.monotonic() - wall0, gap)
                if mode == "intercept" and gap <= within:
                    hit = (_time.monotonic() - wall0, gap)
                    break
                ref_e, ref_n, ff_ve, ff_vn = trk.control_ref(
                    mode, me[0], me[1], tp[0], tp[1], est,
                    min(speed, 0.5 + 1.5 * (_time.monotonic() - wall0)),
                    so_e, so_n, range_m=range_m, orbit=orb)
                # d2 regression: an observation-LESS provider (truth-fed
                # GzPoses) has no ToF beam to serve — stream control_ref's
                # DIRECT reference (target+standoff, velocity feedforward) at
                # the commanded alt: the proven July 6 law. The shaped servo
                # and beam-geometry altitude profile below exist for the
                # camera-fed M3b lane only and stay byte-identical there.
                # W3a: ORBIT always takes this direct lane, camera-fed or
                # not — the shaper below re-derives the feedforward as
                # est + KP·err and DROPS control_ref's tangential term (the
                # carrot would corner-cut the circle), and its altitude
                # profile would descend toward the 2.3 m floor. W3 codex R3:
                # hold_altitude shadow joins this lane even camera-fed — the
                # 1 m/s^2 shaper cannot hold the ring through a mover's 90deg
                # corner (w3-run3.md's 15.1 m corner cut); only the mover
                # default (hold_altitude=False) keeps M3b semantics for
                # shadow/intercept, byte-identical.
                beam_capable = callable(getattr(self.contacts, "observation", None))
                if (mode == "orbit" or (mode == "shadow"
                                        and (hold_altitude or not beam_capable))):
                    ref_u = trk.clamp_ref_alt(self.world, ref_e, ref_n, alt)
                    # yaw: prefer the measured camera bearing (image truth —
                    # the shaper lane's precedence below) so the level camera
                    # CENTERS the target; the 0.4 s predicted lead stays the
                    # fallback for an observation-less provider.
                    obs_fn = getattr(self.contacts, "observation", None)
                    obs = obs_fn(name) if callable(obs_fn) else None
                    mb = getattr(obs, "bearing_deg", None)
                    if mb is not None:
                        yaw = float(mb)
                    else:
                        ly = tp
                        if est.ready:
                            ly = (tp[0] + est.ve * 0.4, tp[1] + est.vn * 0.4)
                        yaw = math.degrees(
                            math.atan2(ly[0] - me[0], ly[1] - me[1]))
                    if hold_altitude:
                        # image-edge barrier (W3 codex R8, the demo shadow
                        # only): the level camera cannot pitch down to follow
                        # a depression transient (w3-run7 K2 — the box bottom
                        # hit row 359/360, then dets=[] and LOST), so as the
                        # designated contact's bbox bottom nears the 360-row
                        # frame floor the guard radius grows by up to 4 m and
                        # the radial reference projects out to it — the
                        # pursuit backs off enough to keep the whole box in
                        # frame. q ramps 0->1 over the bottom 60 px; a stale
                        # view (age_s >= 0.3) or no bbox (bearing-only) keeps
                        # q=0 — no effect, and orbit/truth-fed lanes never
                        # see it.
                        r_vis = r_guard
                        if mode == "shadow":
                            bbox = getattr(obs, "bbox_xyxy", None)
                            age = float(getattr(obs, "age_s", 0.0) or 0.0)
                            if bbox is not None and age < 0.3:
                                q = (float(bbox[3]) - 300.0) / 40.0
                                q = min(1.0, max(0.0, q))
                                if q > 0.0:
                                    r_vis = r_guard + 4.0 * q
                                    dr_e = ref_e - tp[0]
                                    dr_n = ref_n - tp[1]
                                    dr = math.hypot(dr_e, dr_n)
                                    if 1e-9 < dr < r_vis:
                                        ref_e = tp[0] + r_vis * dr_e / dr
                                        ref_n = tp[1] + r_vis * dr_n / dr
                        if 1e-9 < gap < r_vis:
                            # corner interlock (W3 codex R3): while the live
                            # gap is inside the guard radius, add an outward
                            # radial escape velocity (away from the target)
                            # to the feedforward, then re-cap the vector at
                            # 6 m/s — tangential following yields until
                            # geometry recovers. R8: the visibility expansion
                            # adds its own outward term min(3, R_vis-gap)
                            # BEFORE the same cap.
                            if r_vis > r_guard:
                                vis = min(3.0, r_vis - gap)
                                ff_ve += vis * (me[0] - tp[0]) / gap
                                ff_vn += vis * (me[1] - tp[1]) / gap
                            if gap < r_guard:
                                esc = min(2.0, 0.8 * (r_guard - gap))
                                ff_ve += esc * (me[0] - tp[0]) / gap
                                ff_vn += esc * (me[1] - tp[1]) / gap
                            fv = math.hypot(ff_ve, ff_vn)
                            if fv > 6.0:
                                ff_ve, ff_vn = ff_ve * 6.0 / fv, \
                                    ff_vn * 6.0 / fv
                    pos, vel = _sp(ref_e, ref_n, ref_u, ff_ve, ff_vn, yaw)
                    await self.drone.offboard.set_position_velocity_ned(pos, vel)
                    if beam_capable:
                        # orbit on a camera-fed provider: keep feeding the
                        # fusion context (the shaper lane's feed below never
                        # runs here) — in_fusion_envelope admits orbit under
                        # its own speed clause (beam.py, W3a).
                        lp = self.bridge.latest(
                            f"/px4_{0}/fmu/out/vehicle_local_position")
                        own_sp = (math.hypot(getattr(lp, "vx", 0.0),
                                             getattr(lp, "vy", 0.0))
                                  if lp else 0.0)
                        ctx = getattr(self.contacts, "set_beam_context", None)
                        if callable(ctx):
                            ctx(mode=mode, own_speed_mps=own_sp)
                    await asyncio.sleep(1.0 / trk.CTRL_HZ)
                    continue
                # shaped-velocity servo: control_ref gives the REFERENCE, but
                # raw reference jumps saturate PX4's tilt cap and oscillate —
                # instead servo a virtual point (_shp) toward the reference
                # with a P on the position error plus target-velocity
                # feedforward, speed-capped and accel-limited per tick, and
                # stream THAT. The 0.5 + 1.5t ramp above softens the initial
                # dash so the first seconds don't slam the tilt envelope.
                KP, AMAX = 0.7, 1.0 / trk.CTRL_HZ
                err_e, err_n = ref_e - me[0], ref_n - me[1]
                if est.ready:
                    v_e, v_n = est.ve + KP * err_e, est.vn + KP * err_n
                else:
                    v_e, v_n = KP * err_e, KP * err_n
                sp = math.hypot(v_e, v_n)
                if sp > speed:
                    v_e, v_n = v_e * speed / sp, v_n * speed / sp
                dv = math.hypot(v_e - _shp[2], v_n - _shp[3])
                if dv > AMAX and dv > 1e-9:
                    f = AMAX / dv
                    v_e, v_n = _shp[2] + (v_e - _shp[2]) * f, _shp[3] + (v_n - _shp[3]) * f
                _shp = (_shp[0] + v_e / trk.CTRL_HZ,
                        _shp[1] + v_n / trk.CTRL_HZ, v_e, v_n)
                # keep-out bubble: never aim the streamed point INSIDE 7 m of
                # the mover — an airborne target at co-altitude is a collision
                # course. Inside the bubble, hold the reference on its surface
                # along the shaped-point direction and let the gap close
                # tangentially instead of through the target.
                rx, ry = _shp[0] - tp[0], _shp[1] - tp[1]
                rr = math.hypot(rx, ry)
                if 1e-9 < rr < 7.0:
                    ref_e = tp[0] + rx / rr * 7.0
                    ref_n = tp[1] + ry / rr * 7.0
                else:
                    ref_e, ref_n = _shp[0], _shp[1]
                ff_ve, ff_vn = v_e, v_n
                ref_u = trk.clamp_ref_alt(self.world, ref_e, ref_n, alt)
                obs_fn = getattr(self.contacts, "observation", None)
                obs = obs_fn(name) if callable(obs_fn) else None
                # altitude profile: for a ToF-LOCKED contact the ground-mover
                # floor (2.3 m) would park the beam >1 m above the box and
                # starve the very fusion feeding the pursuit (v19: gz_z
                # climbed out of the 0.6-1.8 m band mid-shadow, fusion died,
                # the estimate ghosted to 30-75 m). Hold co-altitude with the
                # live elevation servo instead — box centre to boresight,
                # proportional, bounded ±1 m around the acquired alt; for
                # non-ToF contacts the original profile is unchanged.
                obs_src = getattr(obs, "range_src", None) if obs else None
                e2 = getattr(obs, "elevation_deg", None) if obs else None
                xy2 = getattr(obs, "bbox_xyxy", None) if obs else None
                if hold_altitude:
                    # operator opt-out (W3 codex §4): hold the commanded alt —
                    # the profile below sagged the COCO demo pursuit toward
                    # 2.3-3 m and the car left the frame floor. clamp_ref_alt
                    # still applies above; the min() shape is unchanged.
                    alt_ref = alt
                elif obs_src == "tof" and e2 is not None and xy2 is not None:
                    from agents.perception.projection import vfov_deg
                    fy2 = (360.0 / 2.0) / math.tan(
                        math.radians(vfov_deg(640, 360)) / 2.0)
                    half2 = math.degrees(
                        math.atan(((xy2[3] - xy2[1]) / 2.0) / fy2))
                    e_c = float(e2) + half2
                    alt_ref = max(0.5, min(alt + 1.0, alt + 0.35 * e_c))
                else:
                    # ease DOWN toward co-altitude as the gap closes: 2.3 m
                    # floor, scaling with the gap, never above the commanded
                    # alt. clamp_ref_alt's building rule still wins (min).
                    alt_ref = min(alt, max(2.3, 0.18 * gap + 1.1))
                ref_u = min(ref_u, alt_ref)
                # yaw: prefer the measured camera bearing (image truth —
                # immune to the EKF's yaw/declination offset) so the nose and
                # the beam track what the detector actually sees; fall back to
                # a 0.4 s velocity lead of the estimated position otherwise.
                yaw = None
                mb = getattr(obs, "bearing_deg", None)  # obs read above (alt)
                if mb is not None:
                    yaw = float(mb)
                else:
                    ly = tp
                    if est.ready:
                        ly = (tp[0] + est.ve * 0.4, tp[1] + est.vn * 0.4)
                    yaw = math.degrees(math.atan2(ly[0] - me[0], ly[1] - me[1]))
                pos, vel = _sp(ref_e, ref_n, ref_u, ff_ve, ff_vn, yaw)
                await self.drone.offboard.set_position_velocity_ned(pos, vel)
                # the FLIGHT layer owns the envelope context, fed the VEHICLE's
                # measured speed (codex-R3: this carried the TARGET's estimate
                # — a wrong gate input that also raced the eval collector).
                lp = self.bridge.latest(
                    f"/px4_{0}/fmu/out/vehicle_local_position")
                own_sp = (math.hypot(getattr(lp, "vx", 0.0),
                                     getattr(lp, "vy", 0.0))
                          if lp else 0.0)
                ctx = getattr(self.contacts, "set_beam_context", None)
                if callable(ctx):
                    ctx(mode=mode, own_speed_mps=own_sp)
                await asyncio.sleep(1.0 / trk.CTRL_HZ)
        finally:
            # leave offboard cleanly even on cancellation; Hold keeps position
            try:
                await asyncio.shield(self.drone.offboard.stop())
            except Exception:
                try:
                    await self.drone.action.hold()
                except Exception:
                    pass

        t_total = _time.monotonic() - wall0
        if lost_txt is not None:
            return f"{self.name} {lost_txt}"
        v = (f"target v≈{est.speed():.1f} m/s ({est.ve:+.1f}E {est.vn:+.1f}N)"
             if est.ready else "target velocity not established")
        if mode == "intercept":
            if hit:
                return (f"{self.name} INTERCEPTED {name} at t+{hit[0]:.0f}s, "
                        f"gap {hit[1]:.1f}m; {v}")
            return (f"{self.name} did NOT close within {within:g}m of {name} "
                    f"in {t_total:.0f}s (min gap {log.min_gap:.1f}m); {v}")
        verb = "orbited" if mode == "orbit" else "shadowed"
        return (f"{self.name} {verb} {name} for {t_total:.0f}s: gap min "
                f"{log.min_gap:.1f}m / mean {log.mean_gap():.1f}m, best "
                f"contiguous ≤{within:g}m: {log.best_dwell:.0f}s; {v}")

    async def face(self, target="") -> str:
        tgt = str(target or "").strip().lower()
        if tgt in COMPASS:
            yaw = COMPASS[tgt]
        else:
            me = self.world.world_xy(self.bridge, 0)
            txy = self.world.resolve_xy(tgt)
            if me is None or txy is None:
                raise ValueError(f"can't resolve target '{tgt}'")
            yaw = perception.yaw_deg_to(me[0], me[1], txy[0], txy[1])
        pos = await anext(self.drone.telemetry.position())
        await self.drone.action.goto_location(pos.latitude_deg, pos.longitude_deg,
                                              pos.absolute_altitude_m, yaw)
        # O5: WAIT for the heading to land (≤6°, 100 × 0.1 s = 10 s cap) — a
        # post-face detect then gets a settled, on-target frame.
        err_last = None
        stream = self.drone.telemetry.heading()
        for _ in range(100):
            await asyncio.sleep(0.1)
            try:
                cur = await asyncio.wait_for(anext(stream), 1.0)
                err = abs((yaw - cur.heading_deg + 180.0) % 360.0 - 180.0)
                err_last = err
                if err <= 6.0:
                    return f"{self.name} facing {tgt} (heading {yaw:.0f}deg)"
            except Exception:
                break
        tail = (f", still {err_last:.0f}deg off after 10s"
                if err_last is not None else "")
        return f"{self.name} turning to face {tgt} (heading {yaw:.0f}deg{tail})"

    async def land(self) -> str:
        await self.drone.action.land()
        for _ in range(30):                      # confirm touchdown (safety)
            await asyncio.sleep(1)
            a = self._alt()
            if a is not None and a < 0.5:
                break
        return f"{self.name} landed"

    async def emergency_hold(self) -> str:
        """Public, idempotent estop surface (ICD §5.4): hold position NOW.
        Safe to call while another tool is in flight; pauses an uploaded
        mission first so nothing resumes autonomously."""
        try:
            await self.drone.mission.pause_mission()
        except Exception:
            pass
        try:
            await self.drone.action.hold()
            return f"{self.name} HOLDING (estop)"
        except Exception as e:
            return f"{self.name} hold failed: {e}"

    async def emergency_land(self) -> str:
        """Public, idempotent estop surface: land in place NOW."""
        try:
            await self.drone.mission.pause_mission()
        except Exception:
            pass
        try:
            await self.drone.action.land()
            return f"{self.name} LANDING (estop)"
        except Exception as e:
            return f"{self.name} estop-land failed: {e}"

    def scan(self) -> str:
        movers = self.contacts.poses() if self.contacts is not None else None
        bearing_only = []
        if self.contacts is not None:
            views = getattr(self.contacts, "all_views", None)
            if callable(views):
                try:
                    bearing_only = [v.name for v in views()
                                    if getattr(v, "position_src", None)
                                    in (None, "none")]
                except Exception:
                    bearing_only = []
        return perception.scan_text(self.world, self.bridge,
                                    mover_poses=movers,
                                    bearing_only=bearing_only)

    async def _halt(self) -> None:
        """Stop the vehicle after a cancelled/timed-out mission: cancelling the
        Python coroutine does NOT stop PX4 flying the already-uploaded mission."""
        try:
            await self.drone.mission.pause_mission()
        except Exception:
            try:
                await self.drone.action.hold()
            except Exception:
                pass

    async def _arm_and_start(self, retries=6, delay=1.5):
        """Arm, then start the uploaded mission — retrying start_mission through the
        transient PX4 'DENIED' that routinely hits the first call right after arm.
        Returns once started; re-raises the last error if every attempt fails."""
        await self.drone.action.arm()
        last = None
        for _ in range(max(1, retries)):
            try:
                await self.drone.mission.start_mission()
                return
            except Exception as e:  # MissionError DENIED (vehicle not ready yet), etc.
                last = e
                await asyncio.sleep(delay)
        raise last

    async def run_mission(self, code: str, timeout=None):
        """Exec a Claude-authored async MAVSDK body in-process; return (is_error, text).

        Namespace: `drone` (live System), `mission_item(**fields)`, `world_to_geo`
        (await world_to_geo(east, north, up) -> GeoPoint), `arm_and_start()` (arm +
        start the uploaded mission, retrying the transient PX4 DENIED), `log(msg)`.
        Claude imports MAVSDK classes itself.
        `timeout` (s) is Claude-set; None -> DEFAULT_MISSION_TIMEOUT_S. On timeout
        the vehicle is halted before the error is returned."""
        logs = []
        ns = {
            "drone": self.drone,
            "mission_item": _mission_item,
            "world_to_geo": self._world_to_geo,
            "arm_and_start": self._arm_and_start,
            "log": logs.append,
        }
        src = "async def _snippet():\n" + textwrap.indent(code or "", "    ")
        t = float(timeout) if timeout is not None else DEFAULT_MISSION_TIMEOUT_S
        try:
            exec(compile(src, "<mission>", "exec"), ns)
            ret = await asyncio.wait_for(ns["_snippet"](), timeout=t)
        except asyncio.TimeoutError:
            await self._halt()
            return True, _result_text(
                logs, f"{self.name}: mission timed out after {t:g}s; vehicle halted")
        except asyncio.CancelledError:
            # We're being cancelled (process/shutdown): cancelling the Python
            # coroutine does NOT stop PX4 flying the uploaded mission. Shield the
            # halt so cancellation during the await can't skip it, then re-raise.
            await asyncio.shield(self._halt())
            raise
        except Exception:
            return True, _result_text(logs, traceback.format_exc())
        body = f"{self.name}: completed (no return value)" if ret is None else str(ret)
        return False, _result_text(logs, body)
