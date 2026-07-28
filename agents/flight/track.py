"""Classical real-time tracking: the pure guidance/estimation logic behind
FlightOps.track. The LLM sets the WHAT (target, mode, altitude, duration,
speed cap); this module computes the per-tick control reference and
FlightOps streams it to PX4 offboard as position + velocity feedforward, so
PX4's own cascade (v_des = v_ff + MPC_XY_P*(p_sp - p)) is the PD law — see
docs/superpowers/specs/2026-07-06-track-primitive-design.md.

No MAVSDK/gz imports: everything is state-in/state-out so the controller is
unit-testable without a sim."""
import math

CTRL_HZ = 10.0
MAX_DURATION_S = 120.0
MAX_SPEED_MPS = 12.0
V_EMA_ALPHA = 0.35     # EMA weight of the newest finite-difference sample


class TargetEstimator:
    """Velocity from finite differences of (sim_t, e, n) samples, EMA-smoothed.
    A repeated stamp (stale gz sample between our ticks) is skipped, never
    treated as zero velocity."""

    def __init__(self) -> None:
        self._last = None                    # (t, e, n)
        self.ve = 0.0
        self.vn = 0.0
        self.ready = False                   # True once one real difference seen

    def update(self, t: float, e: float, n: float) -> None:
        if self._last is None:
            self._last = (t, e, n)
            return
        dt = t - self._last[0]
        if dt <= 1e-3:
            return
        ve = (e - self._last[1]) / dt
        vn = (n - self._last[2]) / dt
        if self.ready:
            self.ve += V_EMA_ALPHA * (ve - self.ve)
            self.vn += V_EMA_ALPHA * (vn - self.vn)
        else:
            self.ve, self.vn = ve, vn
            self.ready = True
        self._last = (t, e, n)

    def feed_direct(self, ve: float, vn: float) -> None:
        """O3: accept a velocity from the contact provider's own filter (the
        VisionContacts CV-EKF), bypassing the finite-difference EMA entirely.
        With GzPoses (velocities()=={}) the EMA path stays the fallback."""
        self.ve, self.vn = float(ve), float(vn)
        self.ready = True

    def speed(self) -> float:
        return math.hypot(self.ve, self.vn)


def intercept_t_go(r_e, r_n, v_e, v_n, s):
    """Time-to-go of the constant-velocity lead intercept: smallest positive
    root of (v.v - s^2) t^2 + 2 (r.v) t + r.r = 0, where r = target - drone
    and s is the drone speed cap. None when no positive root exists (target
    as fast as the cap and never closing)."""
    a = v_e * v_e + v_n * v_n - s * s
    b = 2.0 * (r_e * v_e + r_n * v_n)
    c = r_e * r_e + r_n * r_n
    if c == 0.0:
        return 0.0
    if abs(a) < 1e-9:
        return -c / b if b < -1e-9 else None
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    lo, hi = sorted(((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)))
    if lo > 0.0:
        return lo
    if hi > 0.0:
        return hi
    return None


def control_ref(mode, me_e, me_n, tgt_e, tgt_n, est, speed,
                standoff_e=0.0, standoff_n=0.0):
    """One guidance tick -> (ref_e, ref_n, ff_ve, ff_vn).

    shadow:    ref = target + standoff, feedforward = target velocity
               (PX4's outer P closes the residual -> PD on a moving reference).
    intercept: ref = closed-form lead point, feedforward = speed toward it
               (fire-control geometry recomputed every tick; falls back to a
               full-speed tail-chase while the velocity estimate warms up or
               when no root exists)."""
    if mode == "shadow":
        return (tgt_e + standoff_e, tgt_n + standoff_n, est.ve, est.vn)
    t_go = (intercept_t_go(tgt_e - me_e, tgt_n - me_n, est.ve, est.vn, speed)
            if est.ready else None)
    if t_go is None:
        ref_e, ref_n = tgt_e, tgt_n
    else:
        ref_e = tgt_e + est.ve * t_go
        ref_n = tgt_n + est.vn * t_go
    d = math.hypot(ref_e - me_e, ref_n - me_n)
    if d < 1e-6:
        return ref_e, ref_n, 0.0, 0.0
    return ref_e, ref_n, speed * (ref_e - me_e) / d, speed * (ref_n - me_n) / d


def clamp_ref_alt(world, ref_e, ref_n, alt):
    """Building clamp: a reference inside a footprint below roof+3m is raised
    to roof+3m — pursuit must not wedge the drone into a wall (same rule as
    goto's refusal, but a controller mid-chase clamps instead of erroring)."""
    for b in getattr(world, "buildings", None) or []:
        if (abs(ref_e - b["x"]) <= b["w"] / 2 and abs(ref_n - b["y"]) <= b["d"] / 2
                and alt < b["h"] + 3.0):
            return b["h"] + 3.0
    return alt


class TrackLog:
    """Gap bookkeeping for both modes: min/mean gap and the best CONTIGUOUS
    dwell within `within_m` — horizontal, the same metric as the oracle's
    dwell_moving, so the tool's summary is directly checkable by the LLM."""

    def __init__(self, within_m: float) -> None:
        self.within = within_m
        self.n = 0
        self.min_gap = math.inf
        self._sum = 0.0
        self.best_dwell = 0.0
        self._run_start = None

    def sample(self, t: float, gap: float) -> None:
        self.n += 1
        self._sum += gap
        self.min_gap = min(self.min_gap, gap)
        if gap <= self.within:
            if self._run_start is None:
                self._run_start = t
            self.best_dwell = max(self.best_dwell, t - self._run_start)
        else:
            self._run_start = None

    def mean_gap(self) -> float:
        return self._sum / self.n if self.n else math.inf
