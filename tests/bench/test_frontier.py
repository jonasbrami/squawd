from bench import frontier


ROWS = [
    {"backend": "cpu", "resolution": "640x360", "knee_n": 2, "limiting": "cpu"},
    {"backend": "intel", "resolution": "640x360", "knee_n": 10, "limiting": "igpu"},
    {"backend": "nvidia", "resolution": "640x360", "knee_n": 14, "limiting": "cpu"},
    {"backend": "nvidia", "resolution": "1920x1080", "knee_n": 4, "limiting": "vram"},
]


def test_build_frontier_table():
    t = frontier.build_frontier_table(ROWS)
    assert t["intel"]["640x360"] == {"n": 10, "limit": "igpu"}
    assert t["nvidia"]["1920x1080"] == {"n": 4, "limit": "vram"}


def test_render_markdown_has_cells_and_dash():
    t = frontier.build_frontier_table(ROWS)
    md = frontier.render_markdown(
        t, backends=["cpu", "intel", "nvidia"],
        resolutions=["640x360", "1920x1080"])
    assert "640x360" in md
    assert "10 (igpu)" in md
    assert "4 (vram)" in md
    assert "—" in md          # cpu @ 1920x1080 has no row
