"""Pilot package: the single drone agent (PilotAgent) and its estop arbiter.

NOTE: `PilotAgent` is NOT imported here — agents/pilot/agent.py pulls ROS +
MAVSDK at module scope (runtime-only). Import it explicitly where it runs
(`from agents.pilot.agent import PilotAgent`) or in the sim container.
"""

from agents.pilot.estop import ActiveToolRegistry, estop_supervisor

__all__ = ["ActiveToolRegistry", "estop_supervisor"]
