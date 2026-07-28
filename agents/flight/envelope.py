"""The one enforced safety envelope (design §13 item 3, ICD §5.2).

Constants live HERE, at the enforcement point — no parallel config file (an
unenforced config is worse than none). Two distinct roles, not duplication:
- max_alt_m / max_speed_mps: soft tool-layer pre-checks (legible rejections
  the LLM can re-plan from);
- geofence_*: the hard PX4 layer, derived from this dataclass at connect()
  (PilotAgent sets GF_MAX_HOR_DIST / GF_MAX_VER_DIST FROM these values, so the
  two layers cannot diverge). PX4 param-set failure is degraded, not fatal.

run_mission admission: arbitrary authored code CANNOT be statically checked —
the envelope governs the fixed tools; inside run_mission, PX4's own geofence is
the only hard bound (stated in the tool description).
"""
import math
from dataclasses import dataclass

from agents.flight.errors import InvalidParamError


class EnvelopeViolation(InvalidParamError):
    """An out-of-envelope command at the tool boundary."""


@dataclass(frozen=True)
class Envelope:
    max_alt_m: float = 80.0
    max_speed_mps: float = 12.0
    geofence_radius_m: float = 300.0
    geofence_alt_m: float = 80.0
    center_e: float = 0.0          # launch/home, world frame (set at connect)
    center_n: float = 0.0

    def _check_alt(self, alt: float, task_ceiling_m: float | None) -> None:
        ceiling = self.max_alt_m if task_ceiling_m is None else min(
            self.max_alt_m, task_ceiling_m)
        if alt > ceiling:
            raise EnvelopeViolation(
                f"alt {alt:.0f}m exceeds ceiling {ceiling:.0f}m"
                + ("" if task_ceiling_m is None else
                   f" (task ceiling {task_ceiling_m:.0f}m)"))
        if alt < 0.5:
            raise EnvelopeViolation(f"alt {alt:.1f}m is below ground level")

    def _check_xy(self, e: float, n: float) -> None:
        r = math.hypot(e - self.center_e, n - self.center_n)
        if r > self.geofence_radius_m:
            raise EnvelopeViolation(
                f"point E{e:.0f} N{n:.0f} is {r:.0f}m from home, outside the "
                f"{self.geofence_radius_m:.0f}m geofence radius")


def check_takeoff(env: Envelope, alt: float) -> None:
    env._check_alt(float(alt), None)


def check_goto(env: Envelope, e: float, n: float, alt: float,
               task_ceiling_m: float | None = None) -> None:
    env._check_xy(e, n)
    env._check_alt(alt, task_ceiling_m)


def check_orbit(env: Envelope, e: float, n: float, radius: float, alt: float) -> None:
    # validates the PERIMETER, not just the center (ICD §5.2)
    r = math.hypot(e - env.center_e, n - env.center_n) + abs(radius)
    if r > env.geofence_radius_m:
        raise EnvelopeViolation(
            f"orbit around E{e:.0f} N{n:.0f} r={radius:.0f}m reaches {r:.0f}m "
            f"from home, outside the {env.geofence_radius_m:.0f}m geofence")
    env._check_alt(alt, None)


def check_fly_endpoint(env: Envelope, e: float, n: float, alt: float) -> None:
    env._check_xy(e, n)
    env._check_alt(alt, None)


def check_speed(env: Envelope, speed: float) -> None:
    if speed <= 0:
        raise EnvelopeViolation(f"speed {speed:g} must be positive")
    if speed > env.max_speed_mps:
        raise EnvelopeViolation(
            f"speed {speed:.1f} m/s exceeds the {env.max_speed_mps:.1f} m/s cap")
