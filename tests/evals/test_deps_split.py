"""Deps split (design §3.8, Codex-Mj11): the harness wires flight tools to
flight_contacts — NEVER to oracle_truth — and the tier map carries the Kimi
entries (§5.2, map only; no Kimi runs before M6)."""
import asyncio

from evals.runner import Deps, FleetHarness, model_for


class FakeAgent:
    def __init__(self, i):
        self._system = f"sys{i}"

    async def connect(self):
        pass


def _harness(deps):
    h = FleetHarness(deps, n=1, agent_factory=lambda i: FakeAgent(i))
    asyncio.run(h.systems_list())
    return h


def test_tier_map_gains_kimi_entries():
    assert model_for({"drones": "kimi"}, "drones") == "kimi-for-coding"
    assert model_for({"drones": "kimi3"}, "drones") == "k3"
    assert model_for({"drones": "sonnet"}, "drones") == "claude-sonnet-5"


def test_deps_gzposes_alias_reads_oracle_truth():
    truth = object()
    deps = Deps(world=None, bridge=None, cameras=None, oracle_truth=truth)
    assert deps.gzposes is truth            # back-compat read alias
    assert deps.flight_contacts is None and deps.detector is None


def test_make_ops_wires_flight_contacts_never_oracle_truth():
    flight, truth = object(), object()
    deps = Deps(world=None, bridge=None, cameras=None,
                oracle_truth=truth, flight_contacts=flight)
    ops = _harness(deps)._make_ops()
    assert ops.contacts is flight
    assert ops.contacts is not truth


def test_make_ops_carries_the_production_envelope():
    """M6: eval FlightOps fly with the SAME Envelope the production pilot
    builds (agents/pilot/run.py) — envelope parity, not a bare None."""
    from agents.flight.envelope import Envelope
    deps = Deps(world=None, bridge=None, cameras=None)
    ops = _harness(deps)._make_ops()
    assert isinstance(ops.envelope, Envelope)
    assert ops.envelope == Envelope()      # the production defaults


# ---------- M6: detect_text wiring in the eval client ----------

class _FakePipeline:
    def __init__(self, snap):
        self._snap = snap

    def latest(self):
        return self._snap


class _FakeBridge:
    class _P:
        x = 0.0
        y = 0.0
        z = -12.0
        heading = 0.0
        xy_valid = True

    def latest(self, topic):
        return self._P()


def _world_with_pose():
    from agents.world.model import World
    w = World(path="/nonexistent")
    w.note_pose(9.99, 50.0, 20.0, 12.0, 0.0)
    w.note_pose(10.01, 50.0, 20.0, 12.0, 0.0)
    w.note_attitude(9.99, 0.0, 0.0, 0.0)
    w.note_attitude(10.01, 0.0, 0.0, 0.0)
    return w


def _snap_with_target():
    import time
    from agents.vision.pipeline import PerceptionSnapshot
    from agents.vision.types import Detection
    return PerceptionSnapshot(
        schema_version=1, frame_seq=7, sim_stamp=10.0, frame_w=640,
        frame_h=360, completed_monotonic=time.monotonic() - 0.2,
        dets=[Detection("target", 0.91, (300.0, 340.0, 340.0, 360.0))],
        contacts=[], detector={"healthy": True, "latency_ms": 1.0})


def test_client_for_passes_a_working_detect_text_when_pipeline_present(monkeypatch):
    """The eval client gets the production make_detect_text closure fed by
    deps.pipeline — and it WORKS (queries the pipeline, formats detections)."""
    import agents.pilot.detect_text as dt_mod

    captured = {}
    real = dt_mod.make_detect_text

    def spy(world, bridge, pipeline, i=0):
        captured["args"] = (world, bridge, pipeline)
        return real(world, bridge, pipeline, i)

    monkeypatch.setattr(dt_mod, "make_detect_text", spy)
    world, bridge = _world_with_pose(), _FakeBridge()
    pipeline = _FakePipeline(_snap_with_target())
    deps = Deps(world=world, bridge=bridge, cameras=None, pipeline=pipeline)
    client = _harness(deps).client_for("some-model")
    assert captured["args"] == (world, bridge, pipeline)
    assert "mcp__pilot__detect" in client.options.allowed_tools
    # the wired closure (built by the REAL make_detect_text) actually detects
    detect = real(*captured["args"])
    assert detect(None).startswith("1 detections (frame #7")


def test_client_for_passes_no_detect_text_without_a_pipeline(monkeypatch):
    """Truth-fed lane (pipeline None): the detect tool is NOT registered —
    the production behavior when perception is down, not an invention."""
    import agents.pilot.detect_text as dt_mod

    called = []
    monkeypatch.setattr(dt_mod, "make_detect_text",
                        lambda *a: called.append(a))
    deps = Deps(world=None, bridge=None, cameras=None, pipeline=None)
    client = _harness(deps).client_for("some-model")
    assert called == []
    assert "mcp__pilot__detect" not in client.options.allowed_tools


def test_client_for_appends_strategy_snippet_to_system_prompt():
    deps = Deps(world=None, bridge=None, cameras=None)
    h = _harness(deps)
    h.prompt_append = "STRATEGY SNIPPET BODY"
    client = h.client_for("some-model")
    assert client.options.system_prompt.endswith("STRATEGY SNIPPET BODY")
    h.prompt_append = None
    from agents.flight.tools import PILOT_SYSTEM_PROMPT
    assert h.client_for("some-model").options.system_prompt == PILOT_SYSTEM_PROMPT
