"""vision/backends.py — detector backends (ICD §6.2). Everything cv-ish goes
through frame_to_array (THE one RGB888→BGR conversion site).

Baseline: ColorBlobBackend (stdlib+numpy) and OnnxBackend (onnxruntime).
The optional deep sidecar has its own Ultralytics adapters.
"""
import hashlib
import json
import os

import numpy as np

from agents.core.contact import Frame
from agents.vision.types import BackendError, Detection, rle_encode

def frame_to_array(frame: Frame) -> np.ndarray:
    """Frame.rgb (RGB888 bytes) -> uint8 (H, W, 3) BGR ndarray. THE one
    conversion site — ultralytics/OpenCV consume BGR."""
    a = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(
        frame.height, frame.width, 3)
    return a[:, :, ::-1].copy()


def _rgb_array(frame: Frame) -> np.ndarray:
    return np.frombuffer(frame.rgb, dtype=np.uint8).reshape(
        frame.height, frame.width, 3)


class ColorBlobBackend:
    """Interim detector: color-ratio threshold on the mover orange, half-res
    connected components. Pins cls="target"; attaches a real RLE mask.

    Thresholds tuned against live composite frames (2026-07-21): the rendered
    orange is far muter than the SDF ambient (0.9,0.45,0.1)≈(229,115,26) —
    measured box pixels span shadow face (95,68,30) to sunlit top (204,150,74),
    so r>140/r-g>50 rejected the whole box and the detector saw nothing.
    Discriminators vs the gray world: r-g>15, g-b>20, r-b>45."""
    def __init__(self, hsv_lo: tuple = (80, 50, 15),
                 hsv_hi: tuple = (235, 180, 110), min_area_px: int = 40) -> None:
        # thresholds are (r, g, b) bounds on the ORANGE ratio, not HSV angles
        self._lo, self._hi, self._min_area = hsv_lo, hsv_hi, min_area_px

    def _mask(self, a: np.ndarray) -> np.ndarray:
        r = a[:, :, 0].astype(np.int16)
        g = a[:, :, 1].astype(np.int16)
        b = a[:, :, 2].astype(np.int16)
        return ((r >= self._lo[0]) & (r <= self._hi[0])
                & (g >= self._lo[1]) & (g <= self._hi[1])
                & (b >= self._lo[2]) & (b <= self._hi[2])
                & (r - g > 15) & (g - b > 20) & (r - b > 45))

    def infer(self, frame: Frame, conf: float) -> list[Detection]:
        a = _rgb_array(frame)
        m = self._mask(a)[::2, ::2]              # half-res for speed
        dets = []
        for (y1, x1, y2, x2), area in _components(m, self._min_area // 4):
            full = (x1 * 2, y1 * 2, x2 * 2, y2 * 2)
            c = min(0.99, area / (area + 200.0))
            if c < conf:
                continue
            # mask at FULL box resolution (the Detection.mask contract is a
            # box-region RLE at the xyxy scale; half-res encoding has the wrong
            # shape for consumers.
            sub = m[y1:y2, x1:x2]
            dets.append(Detection("target", c, full,
                                  rle_encode(np.kron(sub, np.ones((2, 2),
                                                                  dtype=bool)))))
        return dets


def _components(mask: np.ndarray, min_area: int):
    """Connected components of a bool grid (DFS, no scipy): yields
    ((y1, x1, y2, x2), area_px) for components >= min_area."""
    seen = np.zeros_like(mask, dtype=bool)
    h, w = mask.shape
    for sy in range(h):
        for sx in range(w):
            if not mask[sy, sx] or seen[sy, sx]:
                continue
            stack, seen[sy, sx] = [(sy, sx)], True
            x1 = x2 = sx
            y1 = y2 = sy
            area = 0
            while stack:
                y, x = stack.pop()
                area += 1
                x1, x2 = min(x1, x), max(x2, x)
                y1, y2 = min(y1, y), max(y2, y)
                for ny, nx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] \
                            and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if area >= min_area:
                yield (y1, x1, y2 + 1, x2 + 1), area


class OnnxBackend:
    """YOLO nano via onnxruntime (MIT) — no ultralytics import at runtime.
    Model ships with a SHA-256 manifest (ICD §6.2); mismatch raises BackendError."""
    def __init__(self, model_path: str, manifest_path: str,
                 device: str = "cpu") -> None:
        self.model_path, self.manifest_path, self.device = model_path, manifest_path, device
        self._session = None
        self._classes: tuple = ()        # manifest "classes" table, when present

    def _verify(self) -> dict:
        try:
            manifest = json.load(open(self.manifest_path))
        except Exception as e:
            raise BackendError(f"manifest unreadable: {e}")
        want = manifest.get("sha256")
        if not want:
            raise BackendError("manifest has no sha256")
        h = hashlib.sha256()
        with open(self.model_path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        if h.hexdigest() != want:
            raise BackendError("model sha256 mismatch with manifest")
        return manifest

    def load(self) -> None:
        manifest = self._verify()
        import onnxruntime as ort                # lazy: baseline dep
        try:
            self._session = ort.InferenceSession(
                self.model_path, providers=["CPUExecutionProvider"])
        except Exception as e:
            raise BackendError(f"onnx load failed: {e}")
        self._layout = manifest.get("output", {}).get("layout", "1x84x8400")
        # class names come from the manifest's "classes" table when it ships
        # one (coco-nano-seg-v1's 80 COCO names); the mover manifest lacks it
        # and _decode_seg falls back to its 2-class default
        self._classes = tuple(manifest.get("classes") or ())
        # the exported graph fixes the input size (416 for the latency budget)
        self._input_size = int(self._session.get_inputs()[0].shape[-1])

    def infer(self, frame: Frame, conf: float) -> list[Detection]:
        if self._session is None:
            self.load()
        inp, scale, pad = _letterbox(frame, size=self._input_size)
        outs = self._session.run(None, {"images": inp})
        return _decode_seg(outs, conf, scale, pad, frame.width, frame.height,
                           self._input_size, names=self._classes)


# ---- seg decode helpers (pure numpy — unit-tested without onnxruntime) ----

def _letterbox(frame: Frame, size: int = 640):
    """RGB frame -> (1,3,size,size) float32 /255, aspect kept, gray pad.
    Returns (input, scale, (pad_x, pad_y)) to undo in decode."""
    import numpy as np
    a = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(
        frame.height, frame.width, 3)
    scale = min(size / frame.width, size / frame.height)
    nw, nh = int(round(frame.width * scale)), int(round(frame.height * scale))
    if (nw, nh) != (frame.width, frame.height):
        from PIL import Image as _I
        a = np.asarray(_I.fromarray(a).resize((nw, nh)))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = a
    x = canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return x, scale, (px, py)


def _nms(boxes: "np.ndarray", scores: "np.ndarray", iou: float = 0.45):
    import numpy as np
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_j = (boxes[order[1:], 2] - boxes[order[1:], 0]) * \
                 (boxes[order[1:], 3] - boxes[order[1:], 1])
        order = order[1:][inter / (area_i + area_j - inter + 1e-9) <= iou]
    return keep


def _decode_seg(outs, conf: float, scale: float, pad: tuple,
                fw: int, fh: int, net_size: int = 640,
                names: tuple = ()) -> list[Detection]:
    """YOLO-seg outputs -> Detections. outs[0]: (1, 4+nc+32, A) det head;
    outs[1]: (1, 32, mh, mw) mask protos. Class-aware NMS. `names` is the
    manifest's class table (empty -> the mover model's 2-class default)."""
    import numpy as np
    det, protos = outs[0][0], outs[1][0]          # (C, A), (32, mh, mw)
    n_ch = det.shape[0]
    n_mask = protos.shape[0]
    nc = n_ch - 4 - n_mask
    if nc <= 0:
        raise BackendError(f"bad det head: {n_ch} ch for {n_mask} protos")
    boxes_cxcywh = det[:4].T                       # (A, 4)
    cls_scores = det[4:4 + nc].T                   # (A, nc)
    coeffs = det[4 + nc:].T                        # (A, 32)
    cls_id = cls_scores.argmax(1)
    scores = cls_scores.max(1)
    keep = scores >= conf
    if not keep.any():
        return []
    boxes_cxcywh, scores, cls_id, coeffs = (boxes_cxcywh[keep], scores[keep],
                                            cls_id[keep], coeffs[keep])
    # xywh center -> xyxy, undo letterbox into frame pixels
    x1 = (boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y1 = (boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    x2 = (boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2 - pad[0]) / scale
    y2 = (boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2 - pad[1]) / scale
    boxes = np.stack([x1, y1, x2, y2], 1)
    dets = []
    names = names or ("target", "obstacle")  # mover-nano-seg-v1 classes
    for c in np.unique(cls_id):
        idx = np.where(cls_id == c)[0]
        for i in _nms(boxes[idx], scores[idx]):
            j = idx[i]
            mask = _assemble_mask(coeffs[j], protos, boxes[j], scale, pad,
                                  fw, fh, net_size)
            dets.append(Detection(names[c] if c < len(names) else f"cls_{c}",
                                  float(scores[j]),
                                  (float(boxes[j, 0]), float(boxes[j, 1]),
                                   float(boxes[j, 2]), float(boxes[j, 3])),
                                  mask))
    return dets


def _assemble_mask(coeff, protos, box, scale, pad, fw, fh, net_size=640):
    """coeff @ protos -> sigmoid -> crop to box -> undo letterbox -> frame ->
    RLE. The proto canvas spans the LETTERBOXED net_size x net_size image, so
    it must be upscaled to net_size and de-padded — resizing it straight onto
    the frame stretches across the pad (bug caught in review). Protos are
    cropped to the box BEFORE the matmul (the full-canvas tensordot is the
    decode hotspot, ~20 ms -> ~1 ms)."""
    import numpy as np
    mh, mw = protos.shape[1], protos.shape[2]
    # box in proto coords (frame px * scale + pad, mapped to proto space)
    kx, ky = mw / net_size, mh / net_size
    x1 = int(max(0, (box[0] * scale + pad[0]) * kx))
    y1 = int(max(0, (box[1] * scale + pad[1]) * ky))
    x2 = int(min(mw, (box[2] * scale + pad[0]) * kx) + 1)
    y2 = int(min(mh, (box[3] * scale + pad[1]) * ky) + 1)
    sub = protos[:, y1:y2, x1:x2]                    # (32, bh, bw)
    m = np.tensordot(coeff, sub, axes=(0, 0))        # (bh, bw)
    m = 1.0 / (1.0 + np.exp(-m))
    from PIL import Image as _I
    # proto crop straight to the box's frame-pixel size (box spans the
    # letterbox uniformly, so the crop maps linearly onto the frame box)
    bw, bh = max(1, int(box[2]) - int(box[0])), max(1, int(box[3]) - int(box[1]))
    out = np.asarray(_I.fromarray((m * 255).astype(np.uint8))
                     .resize((bw, bh), _I.BILINEAR)) > 127
    return rle_encode(out)
