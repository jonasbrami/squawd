"""Trajectory math for scripted movers: the single source of truth shared by the
world generator, the in-sim mover plugin, scan, and the oracle cross-check.
Everything here is pure and grid-checked; the sim plugin only adds plumbing."""
import math

import pytest

from agents.world.trajectory import period_s, pos_xy, vel_xy


LINE = {"type": "line", "p0": [0.0, 0.0], "p1": [40.0, 0.0], "speed_mps": 4.0}
LOOP = {"type": "waypoint_loop", "pts": [[0, 0], [30, 0], [30, 40]], "speed_mps": 5.0}
CIRCLE = {"type": "circle", "center": [10.0, -5.0], "radius_m": 20.0, "speed_mps": 2.0}


class TestLine:
    def test_bounces_between_endpoints(self):
        # 40m at 4 m/s: at t=10 it sits at p1, at t=15 halfway back
        assert pos_xy(LINE, 0.0) == (0.0, 0.0)
        assert pos_xy(LINE, 10.0) == pytest.approx((40.0, 0.0))
        assert pos_xy(LINE, 15.0) == pytest.approx((20.0, 0.0))
        assert pos_xy(LINE, 20.0) == pytest.approx((0.0, 0.0))

    def test_period_is_round_trip(self):
        assert period_s(LINE) == pytest.approx(20.0)

    def test_velocity_flips_on_return_leg(self):
        assert vel_xy(LINE, 5.0) == pytest.approx((4.0, 0.0))
        assert vel_xy(LINE, 15.0) == pytest.approx((-4.0, 0.0))

    def test_once_mode_holds_at_end(self):
        once = dict(LINE, mode="once")
        assert pos_xy(once, 100.0) == pytest.approx((40.0, 0.0))
        assert vel_xy(once, 100.0) == pytest.approx((0.0, 0.0))
        assert period_s(once) == math.inf


class TestWaypointLoop:
    def test_visits_waypoints_at_cumulative_times(self):
        # legs: (0,0)->(30,0)=30m, ->(30,40)=40m, ->(0,0)=50m; total 120m @5 = 24s
        assert period_s(LOOP) == pytest.approx(24.0)
        assert pos_xy(LOOP, 0.0) == pytest.approx((0.0, 0.0))
        assert pos_xy(LOOP, 6.0) == pytest.approx((30.0, 0.0))
        assert pos_xy(LOOP, 14.0) == pytest.approx((30.0, 40.0))
        assert pos_xy(LOOP, 24.0) == pytest.approx((0.0, 0.0))

    def test_mid_leg_interpolation(self):
        assert pos_xy(LOOP, 3.0) == pytest.approx((15.0, 0.0))
        # closing leg is the 3-4-5 triangle: velocity (-30,-40)/50 * 5
        assert vel_xy(LOOP, 20.0) == pytest.approx((-3.0, -4.0))

    def test_wraps_past_period(self):
        assert pos_xy(LOOP, 24.0 + 3.0) == pytest.approx(pos_xy(LOOP, 3.0))


class TestCircle:
    def test_starts_east_of_center_and_orbits_ccw(self):
        assert pos_xy(CIRCLE, 0.0) == pytest.approx((30.0, -5.0))
        quarter = period_s(CIRCLE) / 4
        assert pos_xy(CIRCLE, quarter) == pytest.approx((10.0, 15.0))

    def test_period(self):
        assert period_s(CIRCLE) == pytest.approx(2 * math.pi * 20.0 / 2.0)

    def test_speed_is_constant_and_tangent(self):
        vx, vy = vel_xy(CIRCLE, 7.7)
        assert math.hypot(vx, vy) == pytest.approx(2.0)
        # tangent: perpendicular to the radius vector
        px, py = pos_xy(CIRCLE, 7.7)
        assert (px - 10.0) * vx + (py - (-5.0)) * vy == pytest.approx(0.0, abs=1e-9)

    def test_cw_option_reverses(self):
        cw = dict(CIRCLE, ccw=False)
        quarter = period_s(cw) / 4
        assert pos_xy(cw, quarter) == pytest.approx((10.0, -25.0))

    def test_phase0_rotates_start(self):
        shifted = dict(CIRCLE, phase0_deg=90.0)
        assert pos_xy(shifted, 0.0) == pytest.approx((10.0, 15.0))


def test_unknown_type_raises():
    with pytest.raises(ValueError, match="unknown trajectory type"):
        pos_xy({"type": "spline", "speed_mps": 1.0}, 0.0)


def test_negative_time_clamps_to_start():
    assert pos_xy(LOOP, -5.0) == pytest.approx((0.0, 0.0))
