# src/dronebot/perception/gazebo_perception.py
"""v1 perception: subscribe to the sim's RGB + depth camera, derive a coarse
nearest-obstacle reading and the latest JPEG frame, push PerceptionSnapshots
into the PerceptionStore. Easy-sensors-first; swappable later.

Implementation note: Gazebo transport callbacks run on Gazebo's own threads.
We convert + enqueue there (cheap), and a small asyncio poller drains the
queue into the store, so the main event loop is never blocked.
"""
from __future__ import annotations

import asyncio
import time
from queue import Empty, Queue

from dronebot.perception.provider import (
    Obstacle, PerceptionProvider, PerceptionSnapshot,
)

# Gazebo transport + msgs. Import names per Gazebo Harmonic Python bindings.
from gz.transport13 import Node           # type: ignore
from gz.msgs10.image_pb2 import Image      # type: ignore


def _depth_to_obstacles(depth_msg) -> list[Obstacle]:
    """Coarse nearest-obstacle estimate: sample the center patch of the depth
    frame, take the closest finite, positive return; if under the threshold,
    report it as 'ahead'. Refined later (multi-sector, lidar).

    NOTE: assumes a float32 depth image in meters. The exact topic names and
    encoding MUST be confirmed against the running sim with `gz topic -l` and
    `gz topic -e -t <depth_topic> -n 1`; adjust width/height/format if needed.
    """
    import math
    import struct

    width = getattr(depth_msg, "width", 0)
    height = getattr(depth_msg, "height", 0)
    data = bytes(getattr(depth_msg, "data", b""))
    if width <= 0 or height <= 0 or len(data) < width * height * 4:
        return []

    # Sample a small centered patch.
    cx0, cx1 = int(width * 0.45), int(width * 0.55)
    cy0, cy1 = int(height * 0.45), int(height * 0.55)
    nearest = math.inf
    for y in range(cy0, cy1):
        row = y * width
        for x in range(cx0, cx1):
            off = (row + x) * 4
            (d,) = struct.unpack_from("<f", data, off)
            if math.isfinite(d) and d > 0.0 and d < nearest:
                nearest = d

    threshold_m = 15.0
    if nearest < threshold_m:
        return [Obstacle(direction="ahead", distance_m=float(nearest))]
    return []


class GazeboPerception(PerceptionProvider):
    def __init__(self, store, rgb_topic: str, depth_topic: str) -> None:
        self._store = store
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._node = Node()
        self._queue: "Queue[tuple[bytes | None, list[Obstacle]]]" = Queue(maxsize=4)
        self._latest_jpeg: bytes | None = None
        self._latest_obstacles: list[Obstacle] = []
        self._poller: asyncio.Task | None = None
        self._running = False

    def _on_rgb(self, msg: Image) -> None:
        # Store raw bytes; JPEG-encode lazily in the poller thread to keep
        # the Gazebo callback cheap.
        self._latest_jpeg = bytes(msg.data)

    def _on_depth(self, msg) -> None:
        self._latest_obstacles = _depth_to_obstacles(msg)

    async def start(self) -> None:
        self._node.subscribe(Image, self._rgb_topic, self._on_rgb)
        self._node.subscribe(Image, self._depth_topic, self._on_depth)
        self._running = True
        self._poller = asyncio.create_task(self._poll())

    async def _poll(self) -> None:
        while self._running:
            snap = PerceptionSnapshot(
                timestamp=time.monotonic(),
                jpeg_frame=self._latest_jpeg,
                obstacles=list(self._latest_obstacles),
            )
            self._store.update(snap)
            await asyncio.sleep(0.25)  # 4 Hz; perception need not be fast

    async def stop(self) -> None:
        self._running = False
        if self._poller is not None:
            self._poller.cancel()
