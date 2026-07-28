"""tof_fusion_gate — the M3b LIVE gate (runs in the sim container), v4:

  MEASURED GEOMETRY (docs/benchmarks/m3b-status.md + v3 log): mov_1's 1.2 m
  box FLOATS at z in [0.6, 1.8]; the forward ToF reads it to >=12.7 m. Sweet
  spot: physical alt ~1.2 m (box centre) — beam on-box through the whole
  acquisition, box in-FOV down to point-blank. v3 showed the two failure
  modes this version fixes:
    * EKF-z drift GROWS after leaving the pad (-0.04 at takeoff, +0.7..+0.9
      at the rendezvous): the physical hover sinks below the box base. ->
      re-measure the drift against Gz truth every second and re-bias the
      commanded altitude (eval-side bias; the quantity under test is the ToF
      association, not the altitude hold).
    * a PARKED rendezvous dies: the mover orbits away (~60 s lap), contacts
      stay bearing-only. -> chase-rendezvous on the LIVE truth position
      (the M3a-tolerated task-geometry crutch) until within ToF reach, then
      hand over to the DESIGNED flow: designate the bearing-only contact and
      let ops.track's _acquire (image-servo yaw + 2 m/s beam creep) lock it.

  phase 1: chase-rendezvous to <12 m (truth) at physical ~1.0 m — BELOW the
           6° geom threshold at every range >4 m, so the contact at handover
           is bearing-only by construction. (v4 chased at ~1.5-2.6 m during
           the descent, snuck in one geom fix at hd~13.5, and handed over a
           PREDICTED (coasted, stale) contact: the pursuit steered by the
           frozen position — 7-14 m off the orbiting box — yawed the camera
           off the real target, LOST at t+4s. A stale position is worse than
           none: bearing-only hands ops.track the _acquire path — image-
           servo yaw on the FRESH pixel bearing + beam creep -> ToF lock.)
  phase 2: designate + ops.track (ACQUIRING -> RANGE_LOCKED -> shadow at 5 m,
           physical ~1.2 m), collector scores ToF-fused ranges vs GzPoses
           truth (slant p50/p95, availability, false-association).

  docker exec pilot-sim bash -lc 'uv run --no-project python evals/tof_fusion_gate.py'
"""
import asyncio
import math
import os
import statistics
import sys
import time

sys.path.insert(0, "/workspace")

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.rangefinder import GzRangeProvider, RANGE_TOPIC, SimImpairment
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.flight.ops import FlightOps
from agents.perception.projection import pixel_to_angles
from agents.vision.backends import OnnxBackend
from agents.vision.contacts import VisionContacts
from agents.vision.detector import Detector
from agents.vision.pipeline import PerceptionSnapshot


def _publish_snap(bridge, snap_json: str) -> None:
    """Publish the gate's fusion state to /pilot/detections (the M4 demo:
    the cockpit relays this verbatim — SEARCHING -> LOCKED live)."""
    from std_msgs.msg import String
    from agents.core.bus import STATE_QOS
    m = String()
    m.data = snap_json
    bridge.publish("/pilot/detections", String, m, STATE_QOS)


def _snap(contacts, detector, res) -> str:
    views = [v.__dict__ for v in contacts.all_views()]
    return PerceptionSnapshot(
        schema_version=1, frame_seq=res.frame.seq,
        sim_stamp=res.frame.sim_stamp, frame_w=res.frame.width,
        frame_h=res.frame.height,
        completed_monotonic=res.completed_monotonic,
        dets=list(res.detections), contacts=views,
        detector={"healthy": detector.healthy(),
                  "latency_ms": detector.latency_ms()},
        beam=contacts.beam_view(), track=contacts.track_view()).to_json()

TARGET = "mov_1"
DRONE = "x500_depth_0"
CIRCLE_CENTER = (70.0, -100.0)   # movers sidecar: mov_1's orbit (task truth)
COLLECT_S = 45.0
STANDOFF = 8.0          # m — pursuit standoff once locked (FOV margin at
                        # 1.2 m physical, well inside ToF reach)
PHYS_ALT = 1.2          # m — fusion phase: the box centre (box z in [0.6,1.8])
CHASE_ALT = 1.0         # m — chase phase: below the 6 deg geom threshold
                        # beyond ~4 m, so contacts stay bearing-only (fresh
                        # bearings; a coasted geom position goes stale and
                        # mis-aims the pursuit — v4's LOST)
RENDEZVOUS_M = 15.0     # hand over to _acquire HERE: the 10 Hz offboard
                        # hold brakes gently (tilt capped 12 deg, box stays
                        # framed) and the segmented creep closes to ~7 m —
                        # at 20+ m the 1.2 m box is <4° tall and the ±2°
                        # attitude noise rides the erosion margin (EDGE),
                        # starving fusion and coasting the pursuit onto a
                        # ghost (v18: locked but shadowed at mean 22 m).


def die(code: int):
    sys.stdout.flush()
    os._exit(code)      # mavsdk/gz threads abort on a graceful exit path


async def main() -> int:
    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    gz = GzPoses(os.environ.get("GZ_WORLD", "dynamic"), [TARGET, DRONE])
    rec = Px4StateRecorder(bridge, world, i=0, sim_time_ref=gz.sim_time)
    rf = GzRangeProvider(RANGE_TOPIC.format(
        world=os.environ.get("GZ_WORLD", "dynamic")), impair=SimImpairment())
    rf.connect()
    system = System(mavsdk_server_address="127.0.0.1", port=50051)
    contacts = VisionContacts(world, rangefinder=rf)
    ops = FlightOps(system, world, bridge, 0, 1, contacts=contacts)
    # the M2.5 nano-seg artifact, NOT the interim blob: at low level view the
    # blob merges the box's shadowed face with its ground shadow into one
    # component (v16: every elevation/footprint reference contaminated,
    # footprint straddling the eroded top edge => EDGE slips). The trained
    # seg model segments the BOX cleanly (that's what M2.5 is for).
    backend = OnnxBackend("/workspace/models/mover-nano-seg-v1.onnx",
                          "/workspace/models/mover-nano-seg-v1.json")
    detector = Detector(cameras, backend, i=0, hz=10.0, conf=0.2)
    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    # COM_DISARM_LAND 0 (fable-R3, documented SITL demo config, LNDMC stock):
    # PX4's land-detector auto-disarm (2 s default) turns any ground graze
    # into an absorbing failure — offboard setpoints are then ignored and
    # the acquisition dies. Non-zero disarm delay keeps a graze recoverable.
    try:
        await system.param.set_param_int("COM_DISARM_LAND", 0)
        print("COM_DISARM_LAND=0 (graze recoverable)", flush=True)
    except Exception as exc:
        print(f"COM_DISARM_LAND set failed ({exc}) — continuing", flush=True)
    bridge.start()
    rec.start()
    detector.start()
    contacts.attach_detector(detector)

    state = {"res": None}

    async def tick():
        last = 0
        while True:
            res = await asyncio.to_thread(detector.wait_next, after_seq=last,
                                          timeout=0.5)
            if res is not None:
                last = res.frame.seq
                state["res"] = res
                contacts.update(res)
                _publish_snap(bridge, _snap(contacts, detector, res))
            else:
                await asyncio.sleep(0.03)

    ticker = asyncio.create_task(tick())

    print(f"takeoff + warm-up, then chase-rendezvous to <{RENDEZVOUS_M} m …",
          flush=True)
    print(await ops.take_off(2.5), flush=True)
    await ops.tune_pursuit_params()
    print("EKF warm-up 30s…", flush=True)
    await asyncio.sleep(30.0)

    # EKF-z drift, re-measured against Gz truth continuously (EMA); every
    # commanded altitude is biased so the PHYSICAL altitude hits the phase's
    # target. Prime the EMA for 3 s before the chase (v4 started at 0 and
    # lagged the real +1.5 by ~15 s).
    drift = 0.0

    def alt_ekf(phys):
        return phys + drift

    def sample_drift():
        nonlocal drift
        me = world.drone_state(bridge, 0)
        tq = gz.poses().get(DRONE)
        if me and tq:
            drift = 0.7 * drift + 0.3 * (me[2] - tq[2])
        return me, tq

    for _ in range(3):
        sample_drift()
        await asyncio.sleep(1.0)
    print(f"drift primed {drift:+.2f}", flush=True)

    # ---- phase 1: LEAD-INTERCEPT chase on the live truth, LOW, with an
    # image-servo aim assist. Three measured failure modes drive this:
    #   * pure pursuit trails at a ~17 m equilibrium behind a 3.5 m/s mover
    #     (v6) -> aim at tp + v·t_go, cutting the chord;
    #   * a parked wait freezes the yaw and the box leaves the FOV (v5);
    #   * the EKF yaw transient (~25 deg on fresh boots) can point the camera
    #     off the box while contacts can't confirm (v6's views=0 at hd=8-17)
    #     -> when any raw detection exists, servo the yaw off its pixel
    #     bearing (the _acquire trick) instead of the EKF-yaw geometry.
    # Break on a contact inside acquisition reach, bearing-only preferred (a
    # coasted geom position goes stale and mis-aims the pursuit — v4).
    t0 = time.monotonic()
    hd, name, tp_prev, t_prev = None, None, None, None
    while time.monotonic() - t0 < 95.0 and name is None:
        me, tq = sample_drift()
        tp = gz.poses().get(TARGET)
        if not (me and tq and tp):
            await asyncio.sleep(0.5)
            continue
        now = time.monotonic()
        hd = math.hypot(tp[0] - tq[0], tp[1] - tq[1])
        views = contacts.all_views()
        if hd < RENDEZVOUS_M:
            bo = [v for v in views
                  if getattr(v, "position_src", "none") in (None, "none")]
            pick = (bo or views)
            if pick:
                # the FLOATING box sits ABOVE its ground shadow (and the sky/
                # ground glints the orange thresholds also fire on — v13:
                # latching a shadow/glint blob left the footprint on
                # background every time). Pick the track whose footpoint is
                # HIGHEST in the frame (smallest v).
                def _v(v_):
                    fpx = getattr(v_, "foot_px", None)
                    return fpx[1] if fpx is not None else 1e9
                pick.sort(key=_v)
                name = pick[0].name
                break
        # velocity + lead
        vx = vy = 0.0
        if tp_prev is not None and now > t_prev:
            dt = now - t_prev
            vx, vy = (tp[0] - tp_prev[0]) / dt, (tp[1] - tp_prev[1]) / dt
        tp_prev, t_prev = tp, now
        t_go = min(8.0, hd / 7.0)
        lead = (tp[0] + vx * t_go, tp[1] + vy * t_go)
        # RADIAL standoff (v8.8): park on the radius between the orbit's
        # centre and the box's future position, 7 m inside its path — the
        # box then moves TANGENTIALLY relative to the drone (radial speed ~0
        # at closest approach) and the acquisition never fights a receding
        # target (v8.7). 7 m not 10: with ±2° attitude noise the 1.2 m box
        # must be >=4° tall (v17: at 20+ m the beam rides the eroded top
        # edge => EDGE slips; at ~7 m the eroded region absorbs the noise).
        rx, ry = CIRCLE_CENTER[0] - lead[0], CIRCLE_CENTER[1] - lead[1]
        rn = math.hypot(rx, ry) or 1.0
        aim = (lead[0] + 7.0 * rx / rn, lead[1] + 7.0 * ry / rn)
        # leash the goto target with distance: long legs far out (fast
        # transit), ~8 m legs in the last stretch so PX4 decelerates to
        # ~3-4 m/s by the handover — v9.0: an ~8 m/s handover blew through
        # the standoff to hd=1.6 m during _acquire's brake (the box left
        # the FOV right after the first WORLD_TRACKED lock).
        ax_, ay_ = aim[0] - me[0], aim[1] - me[1]
        an = math.hypot(ax_, ay_) or 1.0
        leash = min(an, max(8.0, hd * 0.4))
        tgt = (me[0] + ax_ / an * leash, me[1] + ay_ / an * leash)
        # aim: image-servo when a detection exists, else truth geometry
        dets = state["res"].detections if state["res"] is not None else []
        if dets:
            d0 = dets[0]
            ax, _ay = pixel_to_angles(d0.footpoint[0], d0.footpoint[1],
                                      640, 360)
            yaw = (math.degrees(me[3]) + math.degrees(ax)) % 360
        else:
            yaw = math.degrees(math.atan2(tgt[0] - me[0],
                                          tgt[1] - me[1])) % 360
        try:
            await ops.goto(east=tgt[0], north=tgt[1],
                           up=alt_ekf(CHASE_ALT), heading=f"{yaw:.1f}",
                           wait=False)
        except Exception:
            pass
        el = now - t0
        cands = [(c.cls, c.hits) for c in contacts._candidates]
        res0 = state["res"]
        pose0 = att0 = None
        meas0 = []
        clocks = "no-frame"
        if res0 is not None:
            st0 = res0.frame.sim_stamp
            pose0 = world.pose_at(st0)
            att0 = world.attitude_at(st0)
            meas0 = contacts._measure(res0, pose0, att0)
            pb = world._pose_buf
            clocks = (f"st={st0:.2f} buf=[{pb[0][0]:.2f}..{pb[-1][0]:.2f}] "
                      f"now={gz.sim_time():.2f}") if pb else "buf-empty"
        print(f"  chase t+{el:4.0f}s hd={hd:5.1f} gz_z={tq[2]:.2f} "
              f"drift={drift:+.2f} dets={len(dets)} pose={pose0 is not None} "
              f"att={att0 is not None} meas={[m.kind for m in meas0]} "
              f"cands={cands} tracks={list(contacts._tracks)} | {clocks}",
              flush=True)
        await asyncio.sleep(1.0)
    print(f"rendezvous: hd={hd and round(hd, 1)} m, drift {drift:+.2f} -> "
          f"commanded alt {alt_ekf(PHYS_ALT):.2f} for physical {PHYS_ALT}",
          flush=True)
    if name is None:
        print("no contact in 95s of aimed chase", flush=True)
        die(1)
    # NO separate settle: _acquire's 10 Hz offboard hold + image-servo IS the
    # settle (v8.8: a 1 Hz goto-settle at 7-12 m freezes the yaw between
    # issues and the orbiting box (20-28 deg/s) leaves the FOV in ~1 s).
    v = [v for v in contacts.all_views() if v.name == name][0]
    print(f"contact {name} (position_src="
          f"{getattr(v, 'position_src', '?')}) — designating + tracking",
          flush=True)
    contacts.designate(name)

    # ---- phase 2: the DESIGNED acquisition flow + ToF fusion scoring ----
    errs, fused, frames, false_assoc = [], 0, 0, 0

    async def collect():
        nonlocal fused, frames, false_assoc
        n = 0
        while True:
            frames += 1
            n += 1
            tp = gz.poses().get(TARGET)
            tq = gz.poses().get(DRONE)
            # read-only scoring (codex-R3): the FLIGHT layer owns the beam
            # context with the vehicle's measured speed — the collector must
            # not race it with a fabricated one.
            rr = contacts.ranges().get(name)
            if rr is not None and rr[1] == "tof" and tp and tq:
                fused += 1
                hd2 = math.hypot(tp[0] - tq[0], tp[1] - tq[1])
                slant = math.sqrt(hd2 * hd2 + (tq[2] - tp[2]) ** 2)
                err = abs(rr[0] - slant)
                errs.append(err)
                if err > 3.0:
                    false_assoc += 1
            if n % 5 == 0:
                bl = contacts._beam_last
                s = rf.latest() if callable(getattr(rf, "latest", None)) \
                    else None
                tof = None if s is None else (
                    None if s.range_m is None else round(s.range_m, 2),
                    s.status)
                hd3 = (math.hypot(tp[0] - tq[0], tp[1] - tq[1])
                       if tp and tq else None)
                obs = contacts.observation(name)
                ax_deg = None
                fp = getattr(obs, "foot_px", None) if obs else None
                if fp is not None:
                    _axx, _ayy = pixel_to_angles(fp[0], fp[1], 640, 360)
                    ax_deg = round(math.degrees(_axx), 1)
                el_deg = (round(obs.elevation_deg, 1)
                          if obs and obs.elevation_deg is not None else None)
                dbox = getattr(obs, "bbox_xyxy", None) if obs else None
                dbox = ([round(v) for v in dbox] if dbox else None)
                raw = ([(d.cls, round(d.conf, 2),
                         [round(v) for v in d.xyxy])
                        for d in state["res"].detections]
                       if state["res"] is not None else [])
                print(f"  acq n={n:3d} sm={contacts._sm_state} "
                      f"beam={getattr(bl, 'status', None)}"
                      f"({getattr(bl, 'reason', '')}) "
                      f"tof={tof} ax={ax_deg} el={el_deg} "
                      f"dbox={dbox} dets={raw} "
                      f"hd={hd3 and round(hd3, 1)} "
                      f"gz_z={round(tq[2], 2) if tq else None}", flush=True)
            await asyncio.sleep(0.2)

    collector = asyncio.create_task(collect())
    result = await ops.track(name, mode="shadow", alt=alt_ekf(PHYS_ALT),
                             duration_s=COLLECT_S, within_m=15.0,
                             speed=4.5, standoff_north=-STANDOFF,
                             acquire_budget_s=45.0)
    collector.cancel()
    print("track:", result, flush=True)
    ticker.cancel()
    print(await ops.land(), flush=True)

    avail = fused / frames if frames else 0.0
    errs = sorted(errs)
    p50 = statistics.median(errs) if errs else None
    p95 = errs[int(0.95 * (len(errs) - 1))] if len(errs) >= 2 else None
    far = false_assoc / fused if fused else 0.0
    print(f"\nframes={frames} fused={fused} availability={avail:.2f}")
    print(f"slant error: p50={p50} p95={p95}  false-assoc={far:.3f}")
    ok = (p50 is not None and p50 < 0.5 and p95 < 1.5 and avail >= 0.8
          and false_assoc == 0)
    print(f"M3b TOF GATE (slant <0.5 p50 / <1.5 p95, avail >=80%, "
          f"0 false-assoc): {'PASS' if ok else 'FAIL'}")
    die(0 if ok else 1)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
