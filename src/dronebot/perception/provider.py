# src/dronebot/perception/provider.py
"""Perception modularity contract. The agent and control layers only ever
see PerceptionSnapshot — never the sensor source. Swap GazeboPerception for
a ROS2 / real-sensor provider later without touching anything above.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Obstacle:
    direction: str       # e.g. "ahead", "left", "ahead-left"
    distance_m: float


@dataclass(frozen=True)
class PerceptionSnapshot:
    timestamp: float
    jpeg_frame: bytes | None          # RGB camera frame for the agent's vision
    obstacles: list[Obstacle] = field(default_factory=list)


class PerceptionProvider(ABC):
    @abstractmethod
    async def start(self) -> None:
        """Begin streaming sensor data into the bound store."""

    @abstractmethod
    async def stop(self) -> None:
        """Stop streaming and release sensor resources."""
