"""vision/deep/registry.py — deep-model registry (deep-perception plan §1).

One process, one model instance per name, ONE threading.Lock serializing all
inference (codex R4: YOLOWorld.set_classes() mutates shared embeddings, so
vocabulary switches and predicts ride the same lock; slowlane/on-demand never
queue GPU work). Models load from models/<name>.pt with the repo's manifest
sha256 verification (same pattern as OnnxBackend._verify, ICD §6.2).

The real ultralytics load path is written here (_ultralytics_loader and the
two adapters) but imported lazily INSIDE the loader — this module imports
clean without torch/ultralytics (the M1a gate). Tests inject `loader` fakes;
no GPU anywhere in the test suite.
"""
import hashlib
import json
import os
import re
import threading

import numpy as np

from agents.core.contact import Frame
from agents.vision.types import BackendError, Detection, rle_encode

MAX_PROMPTS = 16                 # wire caps (service maps PromptError -> 422)
MAX_PROMPT_CHARS = 32

# Friendly wire names -> models/<name>.pt (the client's defaults).
ALIASES = {"yolo-world-s": "yolov8s-worldv2", "sam2.1-t": "sam2.1_t"}

_NAME_RE = re.compile(r"[A-Za-z0-9._-]+")   # model name arrives over the wire


class PromptError(ValueError):
    """Prompt vocabulary malformed/over the caps (service maps to 422)."""


def canonical_prompts(prompts) -> tuple:
    """Validate + canonicalize a vocabulary: strip/lower, dedupe, sort
    (cache-friendly for the set_classes pattern). Raises PromptError."""
    if not isinstance(prompts, (list, tuple)) \
            or not all(isinstance(p, str) for p in prompts):
        raise PromptError("prompts must be a list of strings")
    if len(prompts) > MAX_PROMPTS:
        raise PromptError(f"too many prompts ({len(prompts)} > {MAX_PROMPTS})")
    if any(len(p) > MAX_PROMPT_CHARS for p in prompts):
        raise PromptError(f"prompt over {MAX_PROMPT_CHARS} chars")
    vocab = tuple(sorted({p.strip().lower() for p in prompts if p.strip()}))
    if not vocab:
        raise PromptError("prompts need at least one non-empty string")
    return vocab


def mask_result(mask, score: float) -> dict:
    """Full-frame bool mask -> the repo's box-local segment contract (codex
    F8): tight xyxy crop, rle_encode of the crop only, centroid/area in frame
    px. An empty mask is the segment analog of zero dets: all-null fields."""
    a = np.asarray(mask, dtype=bool)
    if a.ndim != 2:
        raise BackendError(f"mask must be 2-D, got shape {a.shape}")
    ys, xs = np.nonzero(a)
    if ys.size == 0:
        return {"xyxy": None, "mask": None, "centroid": None,
                "area_px": 0, "score": 0.0}
    y1, y2 = int(ys.min()), int(ys.max()) + 1
    x1, x2 = int(xs.min()), int(xs.max()) + 1
    return {"xyxy": (float(x1), float(y1), float(x2), float(y2)),
            "mask": rle_encode(a[y1:y2, x1:x2]),
            "centroid": (float(xs.mean()), float(ys.mean())),
            "area_px": int(ys.size),
            "score": float(score)}


class DeepRegistry:
    """Manifest-verified model cache + serialized inference (THE one lock).

    Public detect/segment self-lock. The service instead takes .lock
    non-blocking (429-on-busy) and calls the *_locked variants with the lock
    held — one caller, one style, never both."""

    def __init__(self, models_dir: str = "models", device: str = "cuda",
                 loader=None) -> None:
        self.models_dir, self.device = models_dir, device
        self.lock = threading.Lock()     # the ONE inference lock (codex R4)
        self._loader = loader or _ultralytics_loader
        self._models = {}                # name -> adapter (loads under .lock)
        self._vocab = {}                 # name -> last canonical vocabulary

    # -- loading (always under .lock) --

    def _verify(self, name: str) -> str:
        """Manifest sha256 check (backends.py OnnxBackend._verify pattern);
        returns the verified model path."""
        if not _NAME_RE.fullmatch(name):
            raise BackendError(f"bad model name {name!r}")
        model_path = os.path.join(self.models_dir, name + ".pt")
        try:
            manifest = json.load(
                open(os.path.join(self.models_dir, name + ".json")))
        except Exception as e:
            raise BackendError(f"manifest unreadable: {e}")
        want = manifest.get("sha256")
        if not want:
            raise BackendError("manifest has no sha256")
        h = hashlib.sha256()
        try:
            with open(model_path, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
        except OSError as e:
            raise BackendError(f"model unreadable: {e}")
        if h.hexdigest() != want:
            raise BackendError("model sha256 mismatch with manifest")
        return model_path

    def _load(self, name: str, kind: str):
        name = ALIASES.get(name, name)
        if name not in self._models:
            path = self._verify(name)
            try:
                self._models[name] = self._loader(path, kind, self.device)
            except Exception as e:
                raise BackendError(f"{kind} load failed: {e}")
            self._vocab.pop(name, None)
        return self._models[name]

    def loaded(self) -> list:
        return sorted(self._models)

    def vram_mb(self):
        """Allocated CUDA VRAM in MiB; None without torch/GPU (never raises)."""
        try:
            import torch                            # lazy: M1a gate
            if not torch.cuda.is_available():
                return None
            return round(torch.cuda.memory_allocated() / 2**20)
        except Exception:
            return None

    # -- inference --

    def detect(self, name: str, frame: Frame, prompts, conf: float = 0.25):
        with self.lock:
            return self.detect_locked(name, frame, prompts, conf)

    def detect_locked(self, name: str, frame: Frame, prompts,
                      conf: float = 0.25) -> list:
        """Open-vocab detect -> list[Detection] (boxes only, codex F7). Caller
        holds .lock. set_classes is cached on the canonical vocabulary and
        serialized with predict under the one lock."""
        vocab = canonical_prompts(prompts)
        model = self._load(name, "world")
        key = ALIASES.get(name, name)
        if self._vocab.get(key) != vocab:
            model.set_classes(list(vocab))
            self._vocab[key] = vocab
        return model.detect(frame, conf)

    def segment(self, name: str, frame: Frame, points=None, box=None) -> dict:
        with self.lock:
            return self.segment_locked(name, frame, points=points, box=box)

    def segment_locked(self, name: str, frame: Frame, points=None,
                       box=None) -> dict:
        """One-shot SAM segment -> the box-local mask_result contract. Caller
        holds .lock. Exactly one of points= ([[x,y],...], all positive) or
        box= ([x1,y1,x2,y2])."""
        if (points is None) == (box is None):
            raise PromptError("segment needs exactly one of points= or box=")
        mask, score = self._load(name, "sam").segment(frame, points, box)
        if np.shape(mask) != (frame.height, frame.width):
            raise BackendError(
                f"mask shape {np.shape(mask)} != frame "
                f"{(frame.height, frame.width)}")
        return mask_result(mask, score)


# ---- real load path (lazy heavy imports; fakes replace this in tests) ----

class _WorldModel:
    """ultralytics YOLO-World adapter: set_classes + box-only predict."""

    def __init__(self, model, device: str, to_bgr) -> None:
        self._m, self._dev, self._to_bgr = model, device, to_bgr

    def set_classes(self, vocab: list) -> None:
        m = self._m.model                      # the WorldModel nn.Module
        clip = getattr(m, "clip_model", None)
        if clip is not None:
            # ultralytics 8.4.103 (M1b finding): a CUDA predict() moves the
            # cached CLIP text encoder's weights but not its self.device
            # attribute, so a vocab CHANGE after the first GPU predict
            # tokenizes onto the stale device and crashes (cpu tokens vs
            # cuda weights in F.embedding). Re-pin both before set_classes.
            dev = next(m.parameters()).device
            clip.to(dev)
            clip.device = dev
        self._m.set_classes(vocab)

    def detect(self, frame: Frame, conf: float) -> list:
        res = self._m.predict(self._to_bgr(frame), conf=conf, verbose=False,
                              device=self._dev)[0]
        return [Detection(res.names[int(b.cls)], float(b.conf),
                          tuple(float(v) for v in b.xyxy[0]))
                for b in (res.boxes or [])]


class _SamModel:
    """ultralytics SAM 2.1 one-shot adapter (codex F6: stateless predict with
    points=/bboxes=/labels= — NOT the stateful dynamic predictor)."""

    def __init__(self, model, device: str, to_bgr) -> None:
        self._m, self._dev, self._to_bgr = model, device, to_bgr

    def segment(self, frame: Frame, points, box):
        kw = {"verbose": False, "device": self._dev}
        if points is not None:
            kw["points"] = [[float(x), float(y)] for x, y in points]
            kw["labels"] = [1] * len(points)
        else:
            kw["bboxes"] = [float(v) for v in box]
        res = self._m.predict(self._to_bgr(frame), **kw)[0]
        masks = getattr(res, "masks", None)
        boxes = getattr(res, "boxes", None)
        if masks is None or len(masks) == 0:
            return np.zeros((frame.height, frame.width), dtype=bool), 0.0
        i = int(np.argmax([float(b.conf) for b in boxes])) \
            if boxes is not None and len(boxes) else 0
        score = float(boxes[i].conf) if boxes is not None and len(boxes) \
            else 1.0
        return masks.data[i].cpu().numpy() > 0.5, score


def _ultralytics_loader(path: str, kind: str, device: str):
    """The real load path (the `deep` extra). Lazy by design: THE only
    torch/ultralytics import site in the module, so M1a imports clean without
    them. YOLO() auto-promotes *-worldv2.pt to YOLOWorld (codex F7)."""
    from ultralytics import SAM, YOLO               # noqa: lazy (M1a gate)
    from agents.vision.backends import frame_to_array
    dev = 0 if device == "cuda" else "cpu"
    if kind == "world":
        return _WorldModel(YOLO(path), dev, frame_to_array)
    return _SamModel(SAM(path), dev, frame_to_array)
