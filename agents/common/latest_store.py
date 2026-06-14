"""Thread-safe holder for the most recent value per key.

Written by the rclpy thread, read by the asyncio loop. No torn reads: each
get/set is guarded by a single lock and returns the whole object reference.
"""
import threading
from typing import Any, Optional


class LatestStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._values: dict[str, Any] = {}

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._values[key] = value

    def get(self, key: str) -> Optional[Any]:
        with self._lock:
            return self._values.get(key)
