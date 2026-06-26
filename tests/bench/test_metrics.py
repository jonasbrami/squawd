from bench import metrics


def test_compute_fps_per_drone():
    start = {0: 100, 1: 50}
    end = {0: 160, 1: 80}
    fps = metrics.compute_fps(start, end, dt=6.0)
    assert fps[0] == 10.0      # 60 frames / 6 s
    assert fps[1] == 5.0       # 30 frames / 6 s


def test_compute_fps_zero_dt_is_zero():
    assert metrics.compute_fps({0: 1}, {0: 9}, dt=0.0) == {0: 0.0}


def test_fps_summary():
    s = metrics.fps_summary({0: 10.0, 1: 9.0, 2: 8.0})
    assert s["min"] == 8.0
    assert round(s["mean"], 2) == 9.0
    assert round(s["p10"], 1) == 8.2   # linear-interp 10th pct of [8,9,10]


def test_fps_summary_empty():
    s = metrics.fps_summary({})
    assert s == {"min": 0.0, "mean": 0.0, "p10": 0.0}


def test_verdict_pass():
    v = metrics.evaluate_verdict(fps_min=9.5, cam_fps=10.0, rtf=0.97, alive=4, n=4)
    assert v["pass"] is True
    assert v["reasons"] == []


def test_verdict_fails_on_low_fps():
    v = metrics.evaluate_verdict(fps_min=8.0, cam_fps=10.0, rtf=0.99, alive=4, n=4)
    assert v["pass"] is False
    assert any("fps" in r for r in v["reasons"])


def test_verdict_fails_on_low_rtf():
    v = metrics.evaluate_verdict(fps_min=9.9, cam_fps=10.0, rtf=0.5, alive=4, n=4)
    assert v["pass"] is False
    assert any("rtf" in r for r in v["reasons"])


def test_verdict_fails_on_dead_drone():
    v = metrics.evaluate_verdict(fps_min=9.9, cam_fps=10.0, rtf=0.99, alive=3, n=4)
    assert v["pass"] is False
    assert any("alive" in r for r in v["reasons"])
