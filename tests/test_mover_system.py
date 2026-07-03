"""Pure parts of the gz mover plugin (the gz-bound parts run only in-sim)."""
import datetime
import json

import pytest

from sim.plugins.mover_system import load_spec, to_seconds


def _sidecar(tmp_path, movers):
    p = tmp_path / "dynamic_movers.json"
    p.write_text(json.dumps({"movers": movers}))
    return str(p)


def test_load_spec_finds_mover_by_model_name(tmp_path):
    spec = {"name": "mov_0", "z": 5.0,
            "traj": {"type": "circle", "center": [0, 0], "radius_m": 10, "speed_mps": 1}}
    path = _sidecar(tmp_path, [spec, {"name": "mov_1", "traj": {}}])
    assert load_spec(path, "mov_0") == spec


def test_load_spec_unknown_model_names_the_candidates(tmp_path):
    path = _sidecar(tmp_path, [{"name": "mov_0", "traj": {}}])
    with pytest.raises(KeyError, match="mov_9.*mov_0"):
        load_spec(path, "mov_9")


def test_to_seconds_accepts_timedelta_and_float():
    assert to_seconds(datetime.timedelta(seconds=61, milliseconds=250)) == 61.25
    assert to_seconds(3.5) == 3.5
