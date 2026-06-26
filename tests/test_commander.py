import pytest

# Importing the Commander pulls in rclpy/std_msgs (ROS); skip cleanly where the
# ROS env isn't sourced (host), run it in the container/CI like the ROS tests.
commander = pytest.importorskip("agents.swarm.commander")
CommanderAgent = commander.CommanderAgent


class FakeBridge:
    """Just enough bridge for the TopicLogs the Commander builds in __init__."""
    def subscribe(self, *args, **kwargs):
        pass

    def latest(self, key):
        return None


def _make(monkeypatch, n=2):
    # Avoid constructing a real ClaudeSDKClient (would need the CLI/network).
    monkeypatch.setattr(commander, "make_commander", lambda *a, **k: object())
    return CommanderAgent(n, FakeBridge(), world=None)


def test_skip_replay_backlog_ignores_history(monkeypatch):
    c = _make(monkeypatch, n=2)
    # Simulate the TRANSIENT_LOCAL replay: history already buffered on the cursors.
    c._user.append("old command 1")
    c._user.append("old command 2")
    c._reports[0].append("old report from drone_0")

    c._skip_replay_backlog()

    new_user, _ = c._user.since(c._user_seen)
    new_rep, _ = c._reports[0].since(c._report_seen[0])
    assert new_user == []          # past commands NOT re-executed
    assert new_rep == []           # past reports NOT re-reacted to


def test_new_traffic_after_skip_is_seen(monkeypatch):
    c = _make(monkeypatch, n=2)
    c._user.append("old command")
    c._skip_replay_backlog()

    c._user.append("fresh command")           # arrives AFTER startup
    new_user, c._user_seen = c._user.since(c._user_seen)
    assert new_user == ["fresh command"]       # genuinely new traffic still acted on


def test_skip_is_safe_with_empty_backlog(monkeypatch):
    c = _make(monkeypatch, n=3)
    c._skip_replay_backlog()                   # nothing buffered -> no error, cursors at 0
    assert c._user_seen == 0
    assert c._report_seen == [0, 0, 0]
