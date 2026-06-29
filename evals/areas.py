"""Named world-frame regions (ENU east/north) used by oracle coverage checks.

A region is a polygon (CCW vertices). `area_cells` discretizes it into grid-cell
centers — the denominator for position-overflight coverage. Keep regions here so
task specs stay terse and reusable. Add regions as scenarios need them."""

# ne_quadrant: a 200 m x 200 m box NE of home (home is world origin (0,0)).
AREAS: dict[str, list[tuple[float, float]]] = {
    "ne_quadrant": [(0.0, 0.0), (200.0, 0.0), (200.0, 200.0), (0.0, 200.0)],
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
