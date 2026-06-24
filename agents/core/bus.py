"""RosBridge: run rclpy in a background thread, surface latest msgs to asyncio.

The asyncio side never calls blocking rclpy APIs; it only reads latest() or
publishes. Subscription callbacks run in the rclpy thread, so any callback the
caller passes must be thread-safe (see core.store for the thread-safe holders).
"""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from agents.core.store import LatestStore

# PX4 uXRCE-DDS publishes /fmu/out/* with this profile. Must match to receive.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Swarm chat: reliable + transient_local so late joiners (e.g. the observatory)
# replay the conversation so far.
CHAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)


class RosBridge:
    def __init__(self, node_name: str = "swarm_bridge") -> None:
        rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        self._store = LatestStore()
        self._pubs: dict[str, object] = {}
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def subscribe(self, topic: str, msg_type, qos=PX4_QOS, callback=None) -> None:
        """Subscribe to a topic. Latest msg is always stored; if callback is
        given it is also invoked with each msg (in the rclpy thread)."""
        def _cb(m, t=topic):
            self._store.set(t, m)
            if callback is not None:
                callback(m)
        self._node.create_subscription(msg_type, topic, _cb, qos)

    def publisher(self, topic: str, msg_type, qos=CHAT_QOS):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self._node.create_publisher(msg_type, topic, qos)
            self._pubs[topic] = pub
        return pub

    def publish(self, topic: str, msg_type, msg, qos=CHAT_QOS) -> None:
        self.publisher(topic, msg_type, qos).publish(msg)

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


def publish_str(bridge: RosBridge, topic: str, text: str) -> None:
    """Publish a plain string on `topic` as std_msgs/String over CHAT_QOS.

    The swarm's command/report/chat topics are all latched String channels, so
    both agents wrap their text the same way; this keeps that one-liner in one place.
    """
    m = String()
    m.data = text
    bridge.publish(topic, String, m, CHAT_QOS)
