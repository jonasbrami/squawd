"""The local perception package (ICD §6). Import order rule: trackers ->
types, never the reverse; nothing here imports flight/pilot/ROS/gz."""

from agents.vision.config import VisionConfig, VisionConfigError
from agents.vision.detector import Detector
from agents.vision.types import (AssociationHit, BackendError, Detection,
                                 DetectorBackend, InferenceResult, TrackingMode)

__all__ = ["AssociationHit", "BackendError", "Detection", "Detector",
           "DetectorBackend", "InferenceResult", "TrackingMode", "VisionConfig",
           "VisionConfigError"]
