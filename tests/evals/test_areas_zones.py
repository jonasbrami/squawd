"""Survey-zone ring geometry for the w7 scaling capstone."""
import math

from evals.areas import AREAS, area_cells, point_in_area


def test_eight_zones_on_the_ring():
    for k in range(8):
        poly = AREAS[f"zone_{k}"]
        cx = sum(p[0] for p in poly) / 4
        cy = sum(p[1] for p in poly) / 4
        assert abs(math.hypot(cx, cy) - 130) < 3, (k, cx, cy)
        # 60x60 box
        es = sorted({p[0] for p in poly})
        ns = sorted({p[1] for p in poly})
        assert es[1] - es[0] == 60 and ns[1] - ns[0] == 60


def test_zones_fit_the_geofence_and_do_not_overlap():
    for k in range(8):
        for (e, n) in AREAS[f"zone_{k}"]:
            assert math.hypot(e, n) <= 240
    for k in range(8):
        cells = area_cells(f"zone_{k}", 20.0)
        assert len(cells) == 9      # 3x3 grid of 20m cells in a 60m box
        for (e, n) in cells:
            for j in range(8):
                if j != k:
                    assert not point_in_area(f"zone_{j}", e, n)
