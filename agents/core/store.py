"""Thread-safe holders shared across the rclpy/asyncio boundary. Pure Python
(only threading) so they import — and unit-test — without ROS installed.

- LatestStore: most recent value per key (written by the rclpy thread, read by
  the asyncio loop; whole-object reference, no torn reads).
- TopicLog: append-only history of a std_msgs/String topic. Replaces the
  hand-rolled list+lock+callback the swarm and observatory each used. It calls
  bridge.subscribe but imports no ROS itself, so a fake bridge is enough to test.
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


class TopicLog:
    """Append-only history of a String topic. `since(n)` returns everything after
    index n plus the new length, so a poller can advance its cursor in one call."""

    def __init__(self, bridge, topic: str, msg_type, qos) -> None:
        self._lock = threading.Lock()
        self._items: list[str] = []
        bridge.subscribe(topic, msg_type, qos, self._append)

    def _append(self, m) -> None:
        with self._lock:
            self._items.append(m.data)

    def append(self, text: str) -> None:
        """Add a local-only line (e.g. the observatory echoing your own input)."""
        with self._lock:
            self._items.append(text)

    def all(self) -> list[str]:
        with self._lock:
            return list(self._items)

    def since(self, n: int) -> tuple[list[str], int]:
        with self._lock:
            return self._items[n:], len(self._items)
