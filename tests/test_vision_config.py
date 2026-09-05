"""VisionConfig validation (ICD §0.5/Codex-M8)."""
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


def test_from_env_reads_vars(tmp_path):
    cfg = VisionConfig.from_env({"VISION_BACKEND": "blob",
                                 "VISION_DEVICE": "cpu"})
    assert cfg.backend == "blob"


# ---- W2: admission floors (design 2026-07-28 §4) ----

def test_admit_classes_default_is_v03_allowlist_plus_mover():
    """Default = the §4 dynamic COCO cast + the mover model's two classes (the
    M0→M6 path must keep working on defaults); statics like "chair" are out."""
    cfg = VisionConfig.from_env({})
    assert set(("car", "truck", "bus", "person", "bicycle", "motorcycle")) \
        <= set(cfg.admit_classes)
    assert "target" in cfg.admit_classes and "obstacle" in cfg.admit_classes
    assert "chair" not in cfg.admit_classes
    assert cfg.conf == 0.25


def test_admit_classes_env_override_and_star_disables():
    cfg = VisionConfig.from_env({"VISION_ADMIT_CLASSES": "car, person"})
    assert cfg.admit_classes == ("car", "person")
    assert VisionConfig.from_env(
        {"VISION_ADMIT_CLASSES": "*"}).admit_classes is None


def test_conf_env_parsed_and_fail_closed():
    assert VisionConfig.from_env({"VISION_CONF": "0.3"}).conf == 0.3
    with pytest.raises(VisionConfigError, match="VISION_CONF"):
        VisionConfig.from_env({"VISION_CONF": "bogus"})
    with pytest.raises(VisionConfigError, match="VISION_CONF"):
        VisionConfig.from_env({"VISION_CONF": "1.5"})


# ---- W3: per-model tracker profile (codex §1/§2) ----

def test_coco_models_get_superclass_keys_and_five_second_grace():
    """The shipped coco-* assemblies: vehicle superclass association keys +
    a 5 s lost/rebind grace + the codex-R7 designated-vehicle corner-maneuver
    mode — every OTHER TrackerConfig knob stays the contractual default."""
    for model in ("coco-nano-seg-v1-640.onnx", "coco-nano-seg-v1.onnx"):
        tc = VisionConfig.from_env({"VISION_MODEL": model}).tracker_config()
        assert tc is not None
        assert (tc.lost_s, tc.rebind_window_s) == (5.0, 5.0)
        assert tc.assoc_keys == {"car": "vehicle", "truck": "vehicle",
                                 "bus": "vehicle"}
        assert (tc.gate_m, tc.nis_max, tc.coast_s, tc.birth_hits) == \
            (5.0, 9.21, 1.0, 2)
        # codex R7: maneuver mode activates ONLY here (the "vehicle"
        # superclass), on the maneuver knobs' contractual defaults
        assert tc.maneuver_key == "vehicle"
        assert (tc.maneuver_gate_m, tc.maneuver_trigger_m,
                tc.maneuver_trigger_hits) == (8.0, 1.0, 2)
        assert (tc.maneuver_window_s, tc.maneuver_accel_mps2,
                tc.maneuver_nis_scale) == (2.0, 20.0, 4.0)


def test_mover_and_model_less_selections_keep_the_default_tracker_contract():
    """tracker_config() -> None outside the shipped COCO models: the pilot
    then builds VisionContacts on TrackerConfig's contractual 2.0/2.0
    defaults (the M0→M6 mover path, byte-identical)."""
    assert VisionConfig.from_env({}).tracker_config() is None
    assert VisionConfig.from_env(
        {"VISION_MODEL": "mover-nano-seg-v1.onnx"}).tracker_config() is None
