# src/dronebot/perception/store.py
"""Authoritative latest perception snapshot. Mirrors StateStore."""
from __future__ import annotations

from dronebot.perception.provider import PerceptionSnapshot


class PerceptionStore:
    def __init__(self) -> None:
        self._latest: PerceptionSnapshot | None = None

    def update(self, snapshot: PerceptionSnapshot) -> None:
        self._latest = snapshot

    def latest(self) -> PerceptionSnapshot | None:
        return self._latest

    def surroundings_summary(self) -> str:
        if self._latest is None:
            return "no perception data yet"
        if not self._latest.obstacles:
            return "surroundings clear"
        nearest = min(self._latest.obstacles, key=lambda o: o.distance_m)
        return f"nearest obstacle: {nearest.distance_m:.0f}m {nearest.direction}"
