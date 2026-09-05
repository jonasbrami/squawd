"""Named world-frame regions (ENU east/north) used by oracle coverage checks.

A region is a polygon (CCW vertices). `area_cells` discretizes it into grid-cell
centers — the denominator for position-overflight coverage. Keep regions here so
task specs stay terse and reusable. Add regions as scenarios need them."""

# ne_quadrant: a 200 m x 200 m box NE of home (home is world origin (0,0)).
# ne_block: the am5 no-fly zone — the direct a->b leg cuts through it.
# west_block: the c2 low-survey coverage block.
AREAS: dict[str, list[tuple[float, float]]] = {
    "ne_quadrant": [(0.0, 0.0), (200.0, 0.0), (200.0, 200.0), (0.0, 200.0)],
    "ne_block": [(20.0, 20.0), (180.0, 20.0), (180.0, 180.0), (20.0, 180.0)],
    "west_block": [(-150.0, -50.0), (-50.0, -50.0), (-50.0, 50.0), (-150.0, 50.0)],
    # d3 timing gate (dynamic world): the E110 "fence" is impassable except the
    # N[-10,30] gap that the patrol mover sweeps — these strips make the spatial
    # bypass around the patrol a violation, so timing is the only way through.
    "fence_e110_south": [(105.0, -250.0), (115.0, -250.0), (115.0, -10.0), (105.0, -10.0)],
    "fence_e110_north": [(105.0, 30.0), (115.0, 30.0), (115.0, 250.0), (105.0, 250.0)],
    # w7 survey ring: 8 60x60 zones at r=130, 45deg apart, zone_0 due east.
    # Fleet N surveys zones {k*(8//N)}: N=2 -> E/W, N=4 -> the diagonals too,
    # N=8 -> all. Same world, same geometry, only N scales.
    **{f"zone_{k}": [(cx - 30.0, cy - 30.0), (cx + 30.0, cy - 30.0),
                     (cx + 30.0, cy + 30.0), (cx - 30.0, cy + 30.0)]
       for k, (cx, cy) in enumerate([(130.0, 0.0), (92.0, 92.0), (0.0, 130.0),
                                     (-92.0, 92.0), (-130.0, 0.0), (-92.0, -92.0),
                                     (0.0, -130.0), (92.0, -92.0)])},
}


def _point_in_poly(poly: list[tuple[float, float]], e: float, n: float) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        ei, ni = poly[i]
        ej, nj = poly[j]
        if ((ni > n) != (nj > n)) and (e < (ej - ei) * (n - ni) / (nj - ni) + ei):
            inside = not inside
        j = i
    return inside


def point_in_area(name: str, e: float, n: float) -> bool:
    return _point_in_poly(AREAS[name], e, n)


def area_cells(name: str, cell_m: float) -> list[tuple[float, float]]:
    poly = AREAS[name]
    es = [p[0] for p in poly]
    ns = [p[1] for p in poly]
    e0, e1, n0, n1 = min(es), max(es), min(ns), max(ns)
    cells: list[tuple[float, float]] = []
    e = e0 + cell_m / 2
    while e < e1:
        n = n0 + cell_m / 2
        while n < n1:
            if _point_in_poly(poly, e, n):
                cells.append((e, n))
            n += cell_m
        e += cell_m
    return cells
