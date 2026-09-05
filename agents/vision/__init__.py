"""The local perception package (ICD §6); no flight/pilot/ROS/gz imports."""

from agents.vision.config import VisionConfig, VisionConfigError
from agents.vision.detector import Detector
from agents.vision.types import (AssociationHit, BackendError, Detection,
                                 InferenceResult)

__all__ = ["AssociationHit", "BackendError", "Detection", "Detector",
           "InferenceResult", "VisionConfig",
           "VisionConfigError"]
