"""W1 world buffers: timestamped pose/attitude interpolation, shortest-angle
wrap handling, honest None outside coverage, buffer trimming."""
import math

from agents.world.model import World, _ang_lerp


def world():
    return World(path="/nonexistent")   # falls back to empty cfg


def test_pose_at_interpolates_and_honest_none():
    w = world()
    assert w.pose_at(1.0) is None                     # empty
    w.note_pose(1.0, 0.0, 0.0, 10.0, 0.0)
    assert w.pose_at(0.5) is None                     # before coverage
    assert w.pose_at(2.0) is None                     # after coverage
    w.note_pose(3.0, 10.0, 20.0, 30.0, 1.0)
    e, n, alt, hd = w.pose_at(2.0)
    assert (e, n, alt, hd) == (5.0, 10.0, 20.0, 0.5)
    assert w.pose_at(1.0) == (0.0, 0.0, 10.0, 0.0)    # exact endpoint ok


def test_attitude_at_shortest_angle_wrap():
    w = world()
    w.note_attitude(1.0, 0.1, 0.0, math.pi - 0.1)
    w.note_attitude(3.0, 0.2, 0.0, -math.pi + 0.1)
    roll, pitch, yaw = w.attitude_at(2.0)
    assert abs(roll - 0.15) < 1e-9
    # shortest path +pi-0.1 -> -pi+0.1 goes through +-pi, not through 0
    assert abs(abs(yaw) - math.pi) < 1e-6


def test_ang_lerp_never_takes_the_long_way():
    assert abs(_ang_lerp(math.pi - 0.2, -math.pi + 0.2, 0.5) - math.pi) < 1e-6 \
        or abs(_ang_lerp(math.pi - 0.2, -math.pi + 0.2, 0.5) + math.pi) < 1e-6


def test_buffer_trims_to_window():
    w = world()
    for k in range(10):
        w.note_pose(float(k), float(k), 0.0, 0.0, 0.0)
    assert w.pose_at(0.0) is None                     # trimmed out of the 4s window
    assert w.pose_at(8.0) is not None


def test_ned_to_enu_matches_drone_state_conversion():
    w = World(path="/nonexistent")
    # spawn_x=0, spacing=3: NED(x=5 north, y=-2 west, z=-12 down) ->
    # ENU(east=-2, north=5, alt=12)
    assert w.ned_to_enu(0, 5.0, -2.0, -12.0) == (-2.0, 5.0, 12.0)
