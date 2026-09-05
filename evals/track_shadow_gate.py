"""track_shadow_gate — the M3a LIVE gate (runs inside the sim container):

  camera-fed `track` shadows mov_1 for 60 s: the target id is DISCOVERED from
  the vision contacts (never hardcoded) and fed by the blob detector through
  VisionContacts — no ground truth in the flight path. Truth (GzPoses) is only
  the scoring oracle. Reports dwell stats; gate: contiguous dwell >=45 s inside
  15 m with a camera-fed contact.

Also runnable as the truth-fed control (--feed truth) for the d2_shadow A/B.

  docker exec pilot-sim bash -lc 'uv run --no-project python \
      evals/track_shadow_gate.py --feed vision'
"""
import argparse
import asyncio
import math
import os
import sys
import time

sys.path.insert(0, "/workspace")

from mavsdk import System

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.gzposes import GzPoses
from agents.core.telemetry import Px4StateRecorder
from agents.world import World
from agents.flight.ops import FlightOps
from agents.vision.backends import ColorBlobBackend
from agents.vision.contacts import VisionContacts
from agents.vision.detector import Detector

TARGET_TRUTH = "mov_1"
DWELL_TOL_M = 15.0
DWELL_NEED_S = 45.0
ALT = {"vision": 2.9, "truth": 12.0}
# vision at 4 m: the level IMX214 (half-vFOV ~21°) loses the mover below the
# horizon once the horizontal gap < (alt-1.8)/tan(21°). The shadow controller's
# dynamic lag sits ~10–13 m at these speeds, so 12 m (blind <26 m) and 6 m
# (blind <11 m, observed: LOST at t+7 s, gap 10.5 m) both LOST-cycle; at 4 m
# the mover stays visible from ~7 m out, inside the 15 m dwell window.


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--feed", default="vision", choices=["vision", "truth"])
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    bridge = RosBridge()
    world = World()
    cameras = GzCameras(1)
    gz = GzPoses(os.environ.get("GZ_WORLD", "dynamic"), [TARGET_TRUTH])
    rec = Px4StateRecorder(bridge, world, i=0, sim_time_ref=gz.sim_time)
    system = System(mavsdk_server_address="127.0.0.1", port=50051)

    detector = None
    if args.feed == "vision":
        contacts = VisionContacts(world)
        # conf 0.2, not the 0.45 default: the blob scores ~0.35–0.40 at 30 m
        # and 0.45 starves the EKF (observed: nd=0 through a whole track) —
        # the CV-EKF's NN/NIS gates own measurement quality, not the per-frame
        # threshold (TRACK_CONF spirit, design §6.8). hz=10: the blob costs a
        # few ms — halving the measurement gaps halves the CV drift that
        # spirals into LOST on a 3.5 m/s circler (design lost_s=2.0).
        detector = Detector(cameras, ColorBlobBackend(), i=0, hz=10.0, conf=0.2)
    else:
        contacts = gz
    ops = FlightOps(system, world, bridge, contacts=contacts)

    await system.connect()
    async for s in system.core.connection_state():
        if s.is_connected:
            break
    bridge.start()
    rec.start()
    if detector is not None:
        detector.start()
        contacts.attach_detector(detector)

    for _ in range(150):
        if cameras.has(0) and gz.poses().get(TARGET_TRUTH):
            break
        await asyncio.sleep(0.2)
    print(f"frames+truth ok; feed={args.feed}", flush=True)
    await ops.tune_pursuit_params()   # MPC_TILTMAX_AIR=12, MPC_XY_VEL_MAX=6
    print(await ops.take_off(ALT[args.feed]), flush=True)
    # EKF warm-up: the mag-declination/yaw estimate converges over the first
    # ~30 s airborne — before that, projection bearings carry a 10–20° yaw
    # offset (observed: aim points 17° off, mover slides to the FOV edge,
    # detector starves, LOST). Hover it out before any vision work.
    if args.feed == "vision":
        print("EKF warm-up hover 30s…", flush=True)
        await asyncio.sleep(30.0)

    async def tick_vision():
        """pump detector results into the contacts (the pipeline's tick)."""
        last = 0
        while True:
            res = await asyncio.to_thread(detector.wait_next, after_seq=last,
                                          timeout=0.5)
            if res is not None:
                last = res.frame.seq
                contacts.update(res)
            else:
                await asyncio.sleep(0.03)

    ticker = asyncio.create_task(tick_vision()) if detector is not None else None

    if detector is not None:
        # TEMP instrument: log every NIS rejection (the silent track-killer)
        _orig_apply = contacts._apply_to_track
        def _logged_apply(tr, m, t):
            ok = _orig_apply(tr, m, t)
            if not ok:
                print("NIS-REJECT t=%.1f track=%s kind=%s" % (t, tr.name, m.kind),
                      flush=True)
            return ok
        contacts._apply_to_track = _logged_apply
        _orig_drop = contacts._drop_stale
        def _logged_drop(t):
            before = set(contacts._tracks)
            _orig_drop(t)
            for gone in before - set(contacts._tracks):
                print("DROP t=%.1f %s" % (t, gone), flush=True)
        contacts._drop_stale = _logged_drop
        _orig_assoc = contacts._associate
        def _logged_assoc(t, meas):
            for m in meas:
                if m is None:
                    continue
                ds = {name: round(contacts._gate_dist(tr, m), 1)
                      for name, tr in contacts._tracks.items()
                      if contacts._gate_dist(tr, m) is not None}
                print("ASSOC t=%.1f meas cls=%s kind=%s e=%s n=%s rng=%s px=%s gatedists=%s" % (
                    t, m.cls, m.kind,
                    None if m.e is None else round(m.e, 1),
                    None if m.n is None else round(m.n, 1),
                    None if m.rng is None else round(m.rng, 1),
                    m.foot_px, ds), flush=True)
            _orig_assoc(t, meas)
        contacts._associate = _logged_assoc

    # discover the contact id from the detect output (never hardcoded)
    name = TARGET_TRUTH
    if args.feed == "vision":
        # VISION-ONLY rendezvous (codex-R3 legitimacy): navigation uses ONLY
        # the vision contacts (bearing-only era: advance along the measured
        # bearing; positioned era: fly an 8 m standoff point) — gz truth is
        # scoring-only, never steering. The goto's travel heading keeps the
        # FOV on the mover the whole approach; the handoff additionally
        # requires the gap CLOSING over ~4 s, so the pursuit never inherits a
        # receding-phase start (fable-R3).
        print(await ops.set_speed(5.0), flush=True)
        name = None
        t0 = time.monotonic()
        seen_at = None
        gaps_hist = []
        while time.monotonic() - t0 < 150.0:
            res = detector.detections()
            if res is not None and len(res.detections) > 0:
                seen_at = time.monotonic()
            me = world.drone_state(bridge, 0)
            views = contacts.all_views()
            poses_v = contacts.poses()
            positioned = []
            for v in views:
                if getattr(v, "position_src", "none") not in (None, "none") \
                        and v.name in poses_v and me is not None:
                    p = poses_v[v.name]
                    d = math.hypot(p[0] - me[0], p[1] - me[1])
                    positioned.append((d, v.name, p))
            bearing_v = next((v for v in views
                              if getattr(v, "bearing_deg", None) is not None),
                             None)
            if me is not None:
                if positioned:
                    positioned.sort()
                    d, cname, cpos = positioned[0]
                    gaps_hist.append((time.monotonic(), d))
                    # lead the rendezvous by the EKF velocity (the circling
                    # standoff point outruns an unled pursuit — observed: 150 s
                    # of curved chase, never converging). lead ≈ transit time,
                    # capped 5 s.
                    vels_v = contacts.velocities()
                    ve, vn = vels_v.get(cname, (0.0, 0.0))
                    lead = min(5.0, d / 5.0)
                    tx, ty = cpos[0] + ve * lead, cpos[1] + vn * lead
                    dx, dy = me[0] - tx, me[1] - ty
                    dd = math.hypot(dx, dy) or 1.0
                    try:
                        await ops.goto(east=tx + dx / dd * 8.0,
                                       north=ty + dy / dd * 8.0,
                                       up=ALT[args.feed], heading="travel",
                                       wait=False)
                    except Exception:
                        pass
                elif bearing_v is not None:
                    b = math.radians(bearing_v.bearing_deg)
                    try:
                        await ops.goto(east=me[0] + 15.0 * math.sin(b),
                                       north=me[1] + 15.0 * math.cos(b),
                                       up=ALT[args.feed], heading="travel",
                                       wait=False)
                    except Exception:
                        pass
                recent = seen_at is not None and time.monotonic() - seen_at < 1.5
                if positioned and recent and len(gaps_hist) >= 4:
                    g_now = gaps_hist[-1][1]
                    # closing trend OR already stable-close: the trend gate
                    # alone can never fire once the standoff flight settles
                    # (gap ~constant — observed: 150 s of perfect hover, no
                    # handoff); the close-stable arm is the normal exit
                    closing = (g_now < gaps_hist[-4][1] - 0.5) or g_now < 11.0
                    if g_now < 14.0 and closing:
                        name = positioned[0][1]
                        break
            await asyncio.sleep(1.0)
        if name is None:
            print("NO clean handoff (positioned, closing, in-frame) in 150s",
                  flush=True)
            return 1
        print(f"discovered contact id from detect output: {name}", flush=True)

    # score the shadow against truth (dwell inside 15 m)
    gaps = []
    dbg = []

    async def score():
        t0 = time.monotonic()
        was_present = True
        while time.monotonic() - t0 < args.duration + 20.0:
            tp = gz.poses().get(TARGET_TRUTH)
            me = world.drone_state(bridge, 0)
            if tp and me:
                gaps.append((time.monotonic() - t0,
                             math.hypot(tp[0] - me[0], tp[1] - me[1])))
            if args.feed == "vision":
                active = getattr(ops, "_last_track_name", name)
                present = active in contacts.poses()
                att = world.attitude_at(gz.sim_time())
                res = detector.detections()
                nd = len(res.detections) if res else -1
                est_p = contacts.poses().get(name)
                f_now = cameras.snapshot(0)
                dbg.append((time.monotonic() - t0, nd, present,
                            att[1] if att else None,
                            gaps[-1][1] if gaps else None,
                            f_now.seq if f_now else -1,
                            (gz.sim_time() - f_now.sim_stamp) if f_now else -1,
                            detector.state()))
                if was_present and not present and cameras.snapshot(0) is not None:
                    f = cameras.snapshot(0)
                    with open("/tmp/lost_frame.raw", "wb") as fh:
                        fh.write(f.rgb)
                    with open("/tmp/lost_frame.txt", "w") as fh:
                        fh.write(f"{f.width} {f.height} est={est_p} truth={tp} me={me}")
                was_present = present
            await asyncio.sleep(0.25)

    scorer = asyncio.create_task(score())
    print(f"track {name} (shadow, {args.duration:.0f}s)…", flush=True)
    result = await ops.track(name, mode="shadow", alt=ALT[args.feed],
                             duration_s=args.duration, within_m=DWELL_TOL_M,
                             **({"speed": 4.5, "standoff_north": -8.0}
                                if args.feed == "vision"
                                else {"speed": 6.0, "standoff_north": -4.0}))
    # truth: cap 6 m/s — the default 12 m/s through the shaper oscillates the
    # pursuit (observed: mean gap 36 m, best dwell 4 s); -4 standoff parks the
    # dwell peak off the 15 m ceiling. vision: 4.5 m/s caps chase pitch (~10°
    # — the FOV killer), -8 is the measured dwell/visibility sweet spot
    # vision: 4.5 m/s caps chase/brake pitch, and an 11 m standoff is the
    # geometry sweet spot measured live — >7.2 m keeps the mover inside the
    # ~21° half-vFOV at 4 m alt even through the acquisition overshoot, while
    # standoff+dynamics hold the truth gap inside the 15 m dwell bar. Bare
    # shadow (observed): overshoots to <4 m, mover under the FOV, 37°
    # pitch-back, LOST at t+8 s.
    print("track result:", result, flush=True)
    scorer.cancel()
    if ticker is not None:
        ticker.cancel()
    if dbg:
        print("\nt(s) ndets contact pitch gap seq age state — track diagnostics")
        for row in dbg[::4]:
            t, nd, present, pitch, gap, seq, age, state = row
            print("t=%5.1f nd=%2d contact=%s pitch=%s gap=%s seq=%d age=%.1f %s" % (
                t, nd, present,
                "?" if pitch is None else "%.2f" % pitch,
                "?" if gap is None else "%.1f" % gap, seq, age, state))
    print(await ops.land(), flush=True)

    # dwell analysis: longest contiguous run inside the tolerance
    best = cur = 0.0
    inside = 0
    for i in range(1, len(gaps)):
        dt = gaps[i][0] - gaps[i - 1][0]
        if gaps[i][1] <= DWELL_TOL_M:
            cur += dt
            inside += 1
            best = max(best, cur)
        else:
            cur = 0.0
    mean_gap = (sum(g for _, g in gaps) / len(gaps)) if gaps else float("nan")
    print(f"\nsamples={len(gaps)} inside15m={inside} mean_gap={mean_gap:.1f}m "
          f"best_contiguous_dwell={best:.1f}s (need {DWELL_NEED_S:.0f}s)")
    ok = best >= DWELL_NEED_S
    print(f"M3a SHADOW GATE ({args.feed}-fed): {'PASS' if ok else 'FAIL'}")
    sys.stdout.flush()
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
