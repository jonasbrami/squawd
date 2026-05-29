# src/dronebot/flight_log.py
"""Append-only JSONL flight record: utterance -> tool call -> safety
decision -> result -> telemetry. The experiment's audit trail.
"""
from __future__ import annotations

import json
import time
from typing import Any


class FlightLog:
    def __init__(self, path: str) -> None:
        self._path = path

    def record(self, kind: str, data: dict[str, Any]) -> None:
        entry = {"ts": time.time(), "kind": kind, "data": data}
        with open(self._path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(entry) + "\n")
