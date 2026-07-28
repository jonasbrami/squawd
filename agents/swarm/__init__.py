"""The swarm package is being dismantled in the single-drone rebuild: only
DroneAgent survives for now (the eval FleetHarness reuses its connected System);
agents/pilot/ is the new single-drone agent. Commander + assembler are gone."""

from agents.swarm.drone import DroneAgent

__all__ = ["DroneAgent"]
