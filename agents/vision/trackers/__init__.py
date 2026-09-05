"""vision/trackers/__init__.py — the lazy registry (ICD §6.8).

The single place that knows the taxonomy. Factories lazy-import their
implementation so a missing extra removes the entry without breaking import.
Default is `none` (a first-class no-op entry — it resolves the lock seed,
then the designated contact uses VisionContacts' built-in world-space gate
like every other contact).
"""
import importlib

_REGISTRY = {
    "none":       ("agents.vision.trackers.none", "NoOpTracker", False),
    "botsort":    ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "bytetrack":  ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "ocsort":     ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "deepocsort": ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "tracktrack": ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "fasttrack":  ("agents.vision.trackers.dnn", "DnnAssociationTracker", True),
    "csrt":       ("agents.vision.trackers.template", "CvTemplateTracker", False),
    "kcf":        ("agents.vision.trackers.template", "CvTemplateTracker", False),
    "mosse":      ("agents.vision.trackers.template", "CvTemplateTracker", False),
    "sam2":       ("agents.vision.trackers.sam", "Sam2MaskTracker", False),
}


def _load(name: str):
    module_path, cls_name, needs_ids = _REGISTRY[name]
    mod = importlib.import_module(module_path)
    return getattr(mod, cls_name), needs_ids


def available_trackers(backend=None) -> list[str]:
    """Entries whose implementation imports cleanly AND whose track-id need is
    covered by the backend's supports_track — computed WITHOUT importing
    optional modules at package import time."""
    out = []
    for name in _REGISTRY:
        try:
            cls, needs_ids = _load(name)
        except Exception:
            continue
        if needs_ids and backend is not None \
                and not getattr(backend, "supports_track", False):
            continue
        out.append(name)
    return out


def create_tracker(name: str, device: str = "cpu"):
    """Unknown/unavailable names raise ValueError (validated at assembly)."""
    if name not in _REGISTRY:
        raise ValueError(f"unknown tracker: {name!r}")
    cls, _ = _load(name)
    try:
        return cls(name)
    except TypeError:
        return cls(name, device=device)
