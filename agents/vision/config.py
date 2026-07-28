"""vision/config.py — validated perception configuration (ICD §0.5/Codex-M8).

Explicit selections FAIL CLOSED (VisionConfigError -> sensing-degraded boot);
only `auto` values fall back (with a legible log line).
"""
import os
from dataclasses import dataclass


class VisionConfigError(ValueError):
    pass


@dataclass(frozen=True)
class VisionConfig:
    backend: str = "auto"        # blob | onnx | ultralytics | auto
    weights_dir: str = "models/"
    model: str | None = None
    device: str = "cpu"          # cpu | cuda
    half: bool = False
    tracker: str = "none"        # none | botsort | bytetrack | ... | auto
    tracker_yaml: str | None = None

    @classmethod
    def from_env(cls, env: dict | None = None) -> "VisionConfig":
        e = os.environ if env is None else env
        cfg = cls(
            backend=e.get("VISION_BACKEND", "auto").lower(),
            weights_dir=e.get("VISION_WEIGHTS_DIR", "models/"),
            model=e.get("VISION_MODEL") or None,
            device=e.get("VISION_DEVICE", "cpu").lower(),
            half=e.get("VISION_HALF", "").lower() in ("1", "true", "yes"),
            tracker=e.get("VISION_TRACKER", "none").lower(),
            tracker_yaml=e.get("VISION_TRACKER_YAML") or None,
        )
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if self.backend not in ("blob", "onnx", "ultralytics", "auto"):
            raise VisionConfigError(f"unknown VISION_BACKEND {self.backend!r}")
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
