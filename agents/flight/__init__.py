"""Flight primitives (FlightOps) + their Claude-Agent-SDK tool bindings."""
from agents.flight.ops import FlightOps, COMPASS
from agents.flight.tools import make_pilot_options

__all__ = ["FlightOps", "COMPASS", "make_pilot_options"]
