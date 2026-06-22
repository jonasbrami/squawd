"""Core primitives: ROS transport, geodesy, camera feed.

Import from the submodule you need rather than this package root, so the pure
modules stay usable (and testable) without the heavy sim deps:

    from agents.core.geo import GeoPoint, offset_point   # math only
    from agents.core.store import LatestStore, TopicLog   # threading only
    from agents.core.bus import RosBridge, CHAT_QOS       # needs rclpy
    from agents.core.camera import GzCameras              # needs gz + PIL
"""
