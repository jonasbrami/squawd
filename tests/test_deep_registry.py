"""Deep registry tests (deep-perception plan M1a): manifest sha256
verification failure paths, fake-model detect/segment contracts, the ONE
inference lock's serialization, vocabulary canonicalize+cache, prompt caps,
and the box-local mask crop->RLE contract with empty/malformed handling.
No torch, no GPU — the loader is injected (registry._ultralytics_loader is
the only heavy import site and stays untouched here).
"""
import hashlib
import json
import threading
import time

import numpy as np
import pytest

from agents.core.contact import Frame
from agents.vision.deep.registry import (DeepRegistry, PromptError,
                                         canonical_prompts, mask_result)
from agents.vision.types import BackendError, Detection, rle_decode

WEIGHTS = b"fake weights"


def frame(w=10, h=8):
    return Frame(7, 42.0, w, h, bytes(w * h * 3))


def models_dir(tmp_path, names=("fake-world", "fake-sam")):
    for name in names:
        (tmp_path / f"{name}.pt").write_bytes(WEIGHTS)
        (tmp_path / f"{name}.json").write_text(json.dumps(
            {"sha256": hashlib.sha256(WEIGHTS).hexdigest(), "source": "test"}))
    return str(tmp_path)


class FakeWorld:
    """world-kind fake: records set_classes vocabularies, returns one det."""

    def __init__(self):
        self.vocabs = []
        self.calls = 0

    def set_classes(self, vocab):
        self.vocabs.append(list(vocab))

    def detect(self, f, conf):
        self.calls += 1
        return [Detection("truck", 0.9, (1.0, 2.0, 5.0, 6.0))]


class FakeSam:
    """sam-kind fake: returns a fixed mask (True at rows 2-4, cols 3-6 of an
    8x10 frame) + score, recording the prompt mode."""

    def __init__(self, mask=None, score=0.77):
        self.mask = mask
        self.score = score
        self.calls = []

    def segment(self, f, points, box):
        self.calls.append((points, box))
        if self.mask is not None:
            return self.mask, self.score
        m = np.zeros((f.height, f.width), dtype=bool)
        m[2:5, 3:7] = True
        return m, self.score


def loader(world=None, sam=None, record=None):
    def load(path, kind, device):
        if record is not None:
            record.append((path, kind, device))
        return world if kind == "world" else sam
    return load


def registry(tmp_path, **kw):
    return DeepRegistry(models_dir(tmp_path), **kw)


# ---- manifest verification (the OnnxBackend._verify pattern, ICD §6.2) ----

def test_manifest_missing_is_backend_error(tmp_path):
    d = tmp_path
    (d / "fake-world.pt").write_bytes(WEIGHTS)          # no .json
    reg = DeepRegistry(str(d), loader=loader(FakeWorld()))
    with pytest.raises(BackendError, match="manifest unreadable"):
        reg.detect("fake-world", frame(), ["truck"])


def test_manifest_without_sha256_is_backend_error(tmp_path):
    d = tmp_path
    (d / "fake-world.pt").write_bytes(WEIGHTS)
    (d / "fake-world.json").write_text(json.dumps({"source": "test"}))
    reg = DeepRegistry(str(d), loader=loader(FakeWorld()))
    with pytest.raises(BackendError, match="no sha256"):
        reg.detect("fake-world", frame(), ["truck"])


def test_sha256_mismatch_is_backend_error(tmp_path):
    d = models_dir(tmp_path)
    (tmp_path / "fake-world.pt").write_bytes(b"tampered")
    reg = DeepRegistry(d, loader=loader(FakeWorld()))
    with pytest.raises(BackendError, match="sha256 mismatch"):
        reg.detect("fake-world", frame(), ["truck"])


def test_model_file_missing_is_backend_error(tmp_path):
    d = models_dir(tmp_path)
    (tmp_path / "fake-world.pt").unlink()
    reg = DeepRegistry(d, loader=loader(FakeWorld()))
    with pytest.raises(BackendError, match="model unreadable"):
        reg.detect("fake-world", frame(), ["truck"])


def test_path_traversal_name_rejected(tmp_path):
    reg = registry(tmp_path, loader=loader(FakeWorld()))
    with pytest.raises(BackendError, match="bad model name"):
        reg.detect("../etc/passwd", frame(), ["truck"])


# ---- detect contract + vocabulary canonicalize/cache (codex F7) ----

def test_detect_returns_detections_and_loads_once(tmp_path):
    world, record = FakeWorld(), []
    reg = registry(tmp_path, loader=loader(world, record=record))
    dets = reg.detect("fake-world", frame(), ["truck"], 0.3)
    assert dets == [Detection("truck", 0.9, (1.0, 2.0, 5.0, 6.0))]
    reg.detect("fake-world", frame(), ["truck"], 0.3)
    assert len(record) == 1                          # one instance per name
    kind_path, kind, _dev = record[0]
    assert kind_path.endswith("fake-world.pt") and kind == "world"


def test_vocab_canonicalized_and_cached(tmp_path):
    world = FakeWorld()
    reg = registry(tmp_path, loader=loader(world))
    reg.detect("fake-world", frame(), ["Truck", " truck ", "CAR", "car"])
    reg.detect("fake-world", frame(), ["car", "TRUCK"])     # same canonical
    assert world.vocabs == [["car", "truck"]]        # set_classes ran ONCE
    reg.detect("fake-world", frame(), ["bus"])       # new vocab -> re-set
    assert world.vocabs == [["car", "truck"], ["bus"]]


def test_friendly_alias_resolves_to_model_file(tmp_path):
    d = models_dir(tmp_path, names=("yolov8s-worldv2",))
    record = []
    reg = DeepRegistry(d, loader=loader(FakeWorld(), record=record))
    reg.detect("yolo-world-s", frame(), ["truck"])
    assert record[0][0].endswith("yolov8s-worldv2.pt")


# ---- prompt caps (wire 422 contract, 16 x 32) ----

def test_prompt_caps():
    with pytest.raises(PromptError, match="too many"):
        canonical_prompts([f"p{i}" for i in range(17)])
    with pytest.raises(PromptError, match="32 chars"):
        canonical_prompts(["x" * 33])
    with pytest.raises(PromptError, match="non-empty"):
        canonical_prompts(["", "   "])
    with pytest.raises(PromptError, match="list of strings"):
        canonical_prompts("truck")
    with pytest.raises(PromptError, match="list of strings"):
        canonical_prompts(["truck", 3])
    assert canonical_prompts(["a"] * 16) == ("a",)   # at-cap passes


def test_detect_rejects_over_cap_prompts_before_loading(tmp_path):
    world = FakeWorld()
    reg = registry(tmp_path, loader=loader(world))
    with pytest.raises(PromptError):
        reg.detect("fake-world", frame(), ["x" * 33])
    assert world.calls == 0 and world.vocabs == []


# ---- the ONE inference lock serializes everything (codex R4) ----

def test_one_lock_serializes_inference(tmp_path):
    class SlowWorld(FakeWorld):
        in_flight = 0
        max_in_flight = 0

        def detect(self, f, conf):
            self.in_flight += 1
            self.max_in_flight = max(self.max_in_flight, self.in_flight)
            time.sleep(0.05)
            self.in_flight -= 1
            return []

    world = SlowWorld()
    reg = registry(tmp_path, loader=loader(world))
    threads = [threading.Thread(target=reg.detect,
                                args=("fake-world", frame(), ["a"], 0.5))
               for _ in range(3)]
    t0 = time.monotonic()
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert world.max_in_flight == 1                  # never overlapped
    assert time.monotonic() - t0 >= 0.15             # serialized, not parallel


# ---- segment contract: tight crop + box-local RLE (codex F8) ----

def test_segment_tight_box_and_box_local_rle_round_trip(tmp_path):
    sam = FakeSam()
    reg = registry(tmp_path, loader=loader(sam=sam))
    out = reg.segment("fake-sam", frame(), points=[(4, 3)])
    assert sam.calls == [([(4, 3)], None)]
    assert out["xyxy"] == (3.0, 2.0, 7.0, 5.0)       # tight, half-open
    assert out["area_px"] == 12
    assert out["centroid"] == (4.5, 3.0)
    assert out["score"] == 0.77
    rows = rle_decode(out["mask"], 4, 3)             # box-local dims
    assert rows == [[True] * 4] * 3


def test_segment_box_prompt_path(tmp_path):
    sam = FakeSam()
    reg = registry(tmp_path, loader=loader(sam=sam))
    reg.segment("fake-sam", frame(), box=[0, 0, 9, 7])
    assert sam.calls == [(None, [0, 0, 9, 7])]


def test_segment_needs_exactly_one_prompt_kind(tmp_path):
    reg = registry(tmp_path, loader=loader(sam=FakeSam()))
    with pytest.raises(PromptError, match="exactly one"):
        reg.segment("fake-sam", frame())
    with pytest.raises(PromptError, match="exactly one"):
        reg.segment("fake-sam", frame(), points=[(1, 1)], box=[0, 0, 2, 2])


def test_empty_mask_is_all_null_result():
    out = mask_result(np.zeros((8, 10), dtype=bool), 0.5)
    assert out == {"xyxy": None, "mask": None, "centroid": None,
                   "area_px": 0, "score": 0.0}


def test_malformed_mask_raises_backend_error(tmp_path):
    with pytest.raises(BackendError, match="2-D"):
        mask_result(np.zeros((2, 2, 2), dtype=bool), 0.5)
    wrong_shape = np.zeros((4, 4), dtype=bool)       # != 8x10 frame
    reg = registry(tmp_path, loader=loader(sam=FakeSam(mask=wrong_shape)))
    with pytest.raises(BackendError, match="!= frame"):
        reg.segment("fake-sam", frame(), points=[(1, 1)])


def test_vram_mb_none_without_torch(tmp_path):
    assert registry(tmp_path, loader=loader()).vram_mb() is None
