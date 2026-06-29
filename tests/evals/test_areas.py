import pytest
from evals.areas import AREAS, point_in_area, area_cells


def test_ne_quadrant_registered():
    assert "ne_quadrant" in AREAS


def test_point_inside_and_outside():
    assert point_in_area("ne_quadrant", 50.0, 50.0) is True
    assert point_in_area("ne_quadrant", -50.0, -50.0) is False


def test_unknown_area_raises():
    with pytest.raises(KeyError):
        point_in_area("nowhere", 0.0, 0.0)


def test_area_cells_all_inside_and_nonempty():
    cells = area_cells("ne_quadrant", 20.0)
    assert len(cells) > 0
    assert all(point_in_area("ne_quadrant", e, n) for e, n in cells)
