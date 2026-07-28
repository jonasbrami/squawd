"""Projection contracts (ICD §4.2): pinhole angles in both axes, support-plane
range incl. horizon-None, contact world, box erosion, footprint containment."""
import math

from agents.perception import projection as P


def test_pixel_to_angles_boresight_and_edges():
    ax, ay = P.pixel_to_angles(320, 180, 640, 360)
    assert abs(ax) < 1e-9 and abs(ay) < 1e-9
    ax_r, _ = P.pixel_to_angles(640, 180, 640, 360)
    assert abs(ax_r - math.radians(P.HFOV_DEG / 2)) < 0.02
    _, ay_d = P.pixel_to_angles(320, 360, 640, 360)
    vf = P.vfov_deg(640, 360)
    assert abs(ay_d - math.radians(vf / 2)) < 0.02


def test_vfov_derivation():
    # 2*atan(tan(34.5deg)*360/640) ~ 42deg
    assert abs(P.vfov_deg(640, 360) - 42.0) < 1.5


def test_ray_support_range_nadir_and_45_and_horizon():
    # straight down (level attitude, 20m alt): dep = pi/2 -> range = 20
    r = P.ray_support_range(0.0, math.pi / 2, roll=0.0, pitch=0.0, alt=20.0)
    assert abs(r - 20.0) < 1e-9
    # 45deg depression: slant = 20/sin45 ~ 28.28
    r = P.ray_support_range(0.0, math.pi / 4, roll=0.0, pitch=0.0, alt=20.0)
    assert abs(r - 20.0 / math.sin(math.pi / 4)) < 1e-6
    # at/above horizon -> None (range unobservable)
    assert P.ray_support_range(0.0, 0.0, roll=0.0, pitch=0.0, alt=20.0) is None
    assert P.ray_support_range(0.0, -0.1, roll=0.0, pitch=0.0, alt=20.0) is None
    # support plane above ground
    r = P.ray_support_range(0.0, math.pi / 2, roll=0.0, pitch=0.0, alt=20.0,
                            support_z=5.0)
    assert abs(r - 15.0) < 1e-9
    # plane above the drone -> None
    assert P.ray_support_range(0.0, 1.0, roll=0.0, pitch=0.0, alt=5.0,
                               support_z=10.0) is None


def test_ray_support_range_uses_pitch():
    # dep = ay - pitch (PX4 pitch nose-UP positive). Nose-UP 10° lifts the
    # boresight: a 20° image ray lands at 10° world depression.
    r = P.ray_support_range(0.0, math.radians(20), roll=0.0,
                            pitch=math.radians(10), alt=20.0)
    assert abs(r - 20.0 / math.sin(math.radians(10))) < 1e-6
    # nose-DOWN 10°: the same 20° image ray lands at 30° world depression.
    r = P.ray_support_range(0.0, math.radians(20), roll=0.0,
                            pitch=math.radians(-10), alt=20.0)
    assert abs(r - 20.0 / math.sin(math.radians(30))) < 1e-6


def test_contact_world_bearing_heading():
    # facing north (heading 0), contact straight ahead at 10m -> (0, +10)
    e, n = P.contact_world(5.0, 5.0, 0.0, 0.0, 10.0)
    assert abs(e - 5.0) < 1e-9 and abs(n - 15.0) < 1e-9
    # facing east (heading pi/2), straight ahead -> (+10, 0)
    e, n = P.contact_world(0.0, 0.0, math.pi / 2, 0.0, 10.0)
    assert abs(e - 10.0) < 1e-9 and abs(n) < 1e-9
    # bearing +pi/2 to the right of north -> east
    e, n = P.contact_world(0.0, 0.0, 0.0, math.pi / 2, 10.0)
    assert abs(e - 10.0) < 1e-9 and abs(n) < 1e-9


def test_erode_box_shrinks_by_fraction():
    assert P.erode_box((0, 0, 100, 50), 0.25) == (25.0, 12.5, 75.0, 37.5)


def test_footprint_in_region_disc_vs_box():
    assert P.footprint_in_region((50, 50), 10, (0, 0, 100, 100))
    assert not P.footprint_in_region((15, 50), 20, (0, 0, 100, 100))  # spills left
    assert not P.footprint_in_region((150, 50), 5, (0, 0, 100, 100))
