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


def test_client_for_appends_strategy_snippet_to_system_prompt():
    deps = Deps(world=None, bridge=None, cameras=None)
    h = _harness(deps)
    h.prompt_append = "STRATEGY SNIPPET BODY"
    client = h.client_for("some-model")
    assert client.options.system_prompt.endswith("STRATEGY SNIPPET BODY")
    h.prompt_append = None
    from agents.flight.tools import PILOT_SYSTEM_PROMPT
    assert h.client_for("some-model").options.system_prompt == PILOT_SYSTEM_PROMPT
