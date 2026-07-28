"""VisionConfig validation (ICD §0.5/Codex-M8): explicit selections fail
CLOSED, only auto falls back; tracker/backend pairing is enforced."""
import pytest

from agents.vision.config import VisionConfig, VisionConfigError


def test_unknown_backend_rejected():
    with pytest.raises(VisionConfigError, match="VISION_BACKEND"):
        VisionConfig(backend="magic").validate()


def test_onnx_requires_model_and_weights_dir(tmp_path):
    with pytest.raises(VisionConfigError, match="VISION_MODEL"):
        VisionConfig(backend="onnx").validate()
    with pytest.raises(VisionConfigError, match="weights dir"):
        VisionConfig(backend="onnx", model="x.onnx",
                     weights_dir="/nope/nada").validate()
    VisionConfig(backend="onnx", model="x.onnx",
                 weights_dir=str(tmp_path)).validate()      # ok


def test_blob_needs_no_model():
    VisionConfig(backend="blob").validate()


def test_unknown_tracker_rejected():
    with pytest.raises(VisionConfigError, match="VISION_TRACKER"):
        VisionConfig(tracker="magic").validate()


def test_dnn_tracker_with_blob_fails_closed():
    with pytest.raises(VisionConfigError, match="supports_track"):
        VisionConfig(backend="blob", tracker="botsort").validate()


def test_template_tracker_with_blob_is_fine():
    VisionConfig(backend="blob", tracker="csrt").validate()


def test_from_env_reads_vars(tmp_path):
    cfg = VisionConfig.from_env({"VISION_BACKEND": "blob",
                                 "VISION_TRACKER": "none",
                                 "VISION_DEVICE": "cpu"})
    assert cfg.backend == "blob" and cfg.tracker == "none"
