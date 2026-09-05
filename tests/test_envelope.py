"""Envelope contract (ICD §5.2): legible rejections at the tool boundary —
altitude, speed, geofence radius, orbit perimeter, task ceilings. Constants in
code, no silent clamping."""
import pytest

from agents.flight.envelope import (Envelope, EnvelopeViolation, check_fly_endpoint,
                                    check_goto, check_orbit, check_speed,
                                    check_takeoff, check_track)

ENV = Envelope()


def test_takeoff_rejects_altitude_above_ceiling():
    with pytest.raises(EnvelopeViolation, match="exceeds ceiling"):
        check_takeoff(ENV, 81.0)
    check_takeoff(ENV, 10.0)                      # inside: no raise


def test_track_rejects_pursuit_alt_above_ceiling():
    """M6 commitment guard: the pursuit altitude setpoint obeys the same
    ceiling rule as takeoff."""
    with pytest.raises(EnvelopeViolation, match="exceeds ceiling"):
        check_track(ENV, 90.0)
    with pytest.raises(EnvelopeViolation, match="below ground"):
        check_track(ENV, 0.2)
    check_track(ENV, 12.0)                        # inside: no raise


def test_speed_rejects_over_cap_and_nonpositive():
    with pytest.raises(EnvelopeViolation, match="cap"):
        check_speed(ENV, 12.1)
    with pytest.raises(EnvelopeViolation, match="positive"):
        check_speed(ENV, 0)
    check_speed(ENV, 12.0)


def test_goto_rejects_outside_geofence_radius():
    with pytest.raises(EnvelopeViolation, match="geofence"):
        check_goto(ENV, 400.0, 0.0, 10.0)
    check_goto(ENV, 100.0, 50.0, 10.0)


def test_goto_respects_task_ceiling_below_max_alt():
    with pytest.raises(EnvelopeViolation, match="task ceiling 25"):
        check_goto(ENV, 0.0, 0.0, 30.0, task_ceiling_m=25.0)
    check_goto(ENV, 0.0, 0.0, 20.0, task_ceiling_m=25.0)


def test_orbit_validates_perimeter_not_just_center():
    # center inside, perimeter outside -> reject
    with pytest.raises(EnvelopeViolation, match="reaches"):
        check_orbit(ENV, 250.0, 0.0, radius=60.0, alt=10.0)
    check_orbit(ENV, 100.0, 0.0, radius=60.0, alt=10.0)


def test_fly_endpoint_checked_like_goto():
    with pytest.raises(EnvelopeViolation, match="geofence"):
        check_fly_endpoint(ENV, 0.0, -400.0, 10.0)


def test_envelope_is_frozen_and_centered_configurably():
    env = Envelope(center_e=10.0, center_n=0.0)
    check_goto(env, 10.0 + 299.0, 0.0, 10.0)
    with pytest.raises(EnvelopeViolation):
        check_goto(env, 10.0 + 301.0, 0.0, 10.0)
    with pytest.raises(Exception):                 # FrozenInstanceError
        env.max_alt_m = 1.0
