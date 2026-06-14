"""RosBridge: run rclpy in a background thread, surface latest msgs to asyncio.

The asyncio side never calls blocking rclpy APIs; it only reads latest().
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from agents.common.latest_store import LatestStore

# PX4 uXRCE-DDS publishes /fmu/out/* with this profile. Must match to receive.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class RosBridge:
    def __init__(self, node_name: str = "dronebot_bridge") -> None:
        rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        self._store = LatestStore()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def subscribe(self, topic: str, msg_type) -> None:
        self._node.create_subscription(
            msg_type, topic, lambda m, t=topic: self._store.set(t, m), PX4_QOS
        )

    def latest(self, topic: str):
        """Non-blocking: return the most recent message for topic, or None."""
        return self._store.get(topic)

    def start(self) -> None:
        self._thread.start()

    def _spin(self) -> None:
        rclpy.spin(self._node)

    def shutdown(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()
