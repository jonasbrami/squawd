"""Pure parts of the gz mover plugin (the gz-bound parts run only in-sim)."""
import datetime
import json
import math

import pytest

from sim.plugins.mover_system import (WZ_GAIN, WZ_MAX, link_frame, load_spec,
                                      to_seconds, yaw_drive)


def _sidecar(tmp_path, movers):
    p = tmp_path / "dynamic_movers.json"
    p.write_text(json.dumps({"movers": movers}))
    return str(p)


def test_load_spec_finds_mover_by_model_name(tmp_path):
    spec = {"name": "mov_0", "z": 5.0,
            "traj": {"type": "circle", "center": [0, 0], "radius_m": 10, "speed_mps": 1}}
    path = _sidecar(tmp_path, [spec, {"name": "mov_1", "traj": {}}])
    assert load_spec(path, "mov_0") == spec


def test_load_spec_unknown_model_names_the_candidates(tmp_path):
    path = _sidecar(tmp_path, [{"name": "mov_0", "traj": {}}])
    with pytest.raises(KeyError, match="mov_9.*mov_0"):
        load_spec(path, "mov_9")


def test_to_seconds_accepts_timedelta_and_float():
    assert to_seconds(datetime.timedelta(seconds=61, milliseconds=250)) == 61.25
    assert to_seconds(3.5) == 3.5


def test_yaw_drive_proportional_and_sign():
    """Small errors get a proportional rate, sign chasing the target."""
    assert yaw_drive(0.1, 0.0) == pytest.approx(WZ_GAIN * 0.1)
    assert yaw_drive(0.0, 0.1) == pytest.approx(-WZ_GAIN * 0.1)
    assert yaw_drive(1.0, 1.0) == 0.0


def test_yaw_drive_clamps_large_errors():
    assert yaw_drive(math.radians(179.0), 0.0) == WZ_MAX
    assert yaw_drive(0.0, math.pi / 2) == -WZ_MAX
    assert abs(yaw_drive(math.pi, 0.0)) == WZ_MAX   # 180deg: either way is short


def test_yaw_drive_wraps_across_pi():
    """The short way around: heading +179deg chasing -179deg turns +2deg
    THROUGH the +/-pi seam, not -358deg the long way."""
    err = math.radians(2.0)
    assert yaw_drive(math.radians(-179.0), math.radians(179.0)) == \
        pytest.approx(WZ_GAIN * err)


def test_link_frame_identity_at_yaw_zero():
    """Pre-W1b movers never rotate: the rotation must be a no-op at yaw 0."""
    assert link_frame(3.0, -1.5, 0.0) == pytest.approx((3.0, -1.5))


def test_link_frame_rotates_world_velocity_into_link_frame():
    """gz SetLinearVelocity is LINK-frame: a world -y velocity at yaw -90deg
    (car_1's west leg) must be commanded as +x in the link frame."""
    assert link_frame(0.0, -4.0, -math.pi / 2) == pytest.approx((4.0, 0.0))
    # north leg: world -x at yaw pi -> link +x (nose-first, not backward)
    assert link_frame(-4.0, 0.0, math.pi) == pytest.approx((4.0, 0.0))
