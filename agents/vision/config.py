"""vision/config.py — validated perception configuration (ICD §0.5/Codex-M8).

Explicit selections FAIL CLOSED (VisionConfigError -> sensing-degraded boot);
only `auto` values fall back (with a legible log line).
"""
import os
from dataclasses import dataclass


class VisionConfigError(ValueError):
    pass


# W2 (design 2026-07-28 §4): classes admitted to TRACKABLE contacts — the
# demo's dynamic COCO cast plus the mover model's two classes (the M0→M6 path
# must keep working on defaults). Overlay DISPLAY is unfiltered; anything off
# this list (e.g. "chair") can never birth a contact.
DEFAULT_ADMIT_CLASSES = ("target", "obstacle", "car", "truck", "bus",
                         "person", "bicycle", "motorcycle")

# W3 (codex §1/§2): the shipped COCO models flap car<->truck<->bus at range
# and flicker for seconds at a time (W3 integration: 11/11 demo pursuits
# LOST to contact-id churn inside the 2 s mover contract). Those models
# assemble the contact tracker with ONE "vehicle" association key for the
# flap set plus a 5 s drop/rebind grace; everything else keeps
# TrackerConfig's contractual defaults (tracker_config() -> None).
COCO_ASSOC_SUPERCLASSES = {"car": "vehicle", "truck": "vehicle",
                           "bus": "vehicle"}
COCO_TRACKER_LOST_S = 5.0
# codex R7 (w3-run6's CV-EKF corner ghost): the same COCO profile arms the
# bounded corner-maneuver mode on the designated "vehicle" track — the
# trigger/gate/window knobs stay TrackerConfig's contractual maneuver
# defaults. tracker_config() -> None keeps the mover path with
# maneuver_key=None (OFF, byte-identical).
COCO_MANEUVER_KEY = "vehicle"


@dataclass(frozen=True)
class VisionConfig:
    backend: str = "auto"        # blob | onnx | ultralytics | auto
    weights_dir: str = "models/"
    model: str | None = None
    device: str = "cpu"          # cpu | cuda
    half: bool = False
    tracker: str = "none"        # none | botsort | bytetrack | ... | auto
    tracker_yaml: str | None = None
    conf: float = 0.25           # detector conf floor (post-birth filter)
    admit_classes: tuple | None = DEFAULT_ADMIT_CLASSES  # None = admit all

    @classmethod
    def from_env(cls, env: dict | None = None) -> "VisionConfig":
        e = os.environ if env is None else env
        admit = e.get("VISION_ADMIT_CLASSES")
        try:
            conf = float(e.get("VISION_CONF", "0.25"))
        except ValueError:
            raise VisionConfigError(
                f"unparseable VISION_CONF {e.get('VISION_CONF')!r}")
        cfg = cls(
            backend=e.get("VISION_BACKEND", "auto").lower(),
            weights_dir=e.get("VISION_WEIGHTS_DIR", "models/"),
            model=e.get("VISION_MODEL") or None,
            device=e.get("VISION_DEVICE", "cpu").lower(),
            half=e.get("VISION_HALF", "").lower() in ("1", "true", "yes"),
            tracker=e.get("VISION_TRACKER", "none").lower(),
            tracker_yaml=e.get("VISION_TRACKER_YAML") or None,
            conf=conf,
            admit_classes=(None if admit is not None and admit.strip() in ("*", "")
                           else tuple(c.strip() for c in admit.split(",") if c.strip())
                           if admit is not None else DEFAULT_ADMIT_CLASSES),
        )
        cfg.validate()
        return cfg

    def tracker_config(self):
        """The contact tracker's fusion knobs for THIS model selection (W3
        codex §1/§2, R7): the shipped coco-* models get the vehicle superclass
        association keys + a 5 s lost/rebind grace + the designated-vehicle
        corner-maneuver mode; None = TrackerConfig's contractual defaults
        (the mover M0->M6 path, byte-identical)."""
        if not (self.model or "").startswith("coco-"):
            return None
        from agents.vision.contacts import TrackerConfig
        return TrackerConfig(lost_s=COCO_TRACKER_LOST_S,
                             rebind_window_s=COCO_TRACKER_LOST_S,
                             assoc_keys=dict(COCO_ASSOC_SUPERCLASSES),
                             maneuver_key=COCO_MANEUVER_KEY)

    def validate(self) -> None:
        if self.backend not in ("blob", "onnx", "ultralytics", "auto"):
            raise VisionConfigError(f"unknown VISION_BACKEND {self.backend!r}")
        if not 0.0 < self.conf < 1.0:
            raise VisionConfigError(f"VISION_CONF {self.conf} outside (0, 1)")
        if self.device not in ("cpu", "cuda"):
            raise VisionConfigError(f"unknown VISION_DEVICE {self.device!r}")
        if self.backend in ("onnx", "ultralytics"):
            if not self.model:
                raise VisionConfigError(
                    f"backend {self.backend} requires VISION_MODEL")
            if not os.path.isdir(self.weights_dir):
                raise VisionConfigError(
                    f"weights dir {self.weights_dir!r} does not exist")
        known = ("none", "auto", "botsort", "bytetrack", "ocsort", "deepocsort",
                 "tracktrack", "fasttrack", "csrt", "kcf", "mosse", "sam2")
        if self.tracker not in known:
            raise VisionConfigError(f"unknown VISION_TRACKER {self.tracker!r}")
        if self.tracker != "none" and self.backend == "blob":
            # blob has no track mode: DNN trackers can't pair (fail closed)
            if self.tracker in ("botsort", "bytetrack", "ocsort", "deepocsort",
                                "tracktrack", "fasttrack"):
                raise VisionConfigError(
                    f"tracker {self.tracker} needs track ids but backend "
                    f"'blob' has supports_track=False — use "
                    f"VISION_BACKEND=ultralytics or VISION_TRACKER=none")
