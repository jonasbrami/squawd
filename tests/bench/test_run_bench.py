from bench import run_bench


def test_slice_samples_window():
    s = [{"t": 0.0}, {"t": 5.0}, {"t": 9.9}, {"t": 10.0}, {"t": 12.0}]
    out = run_bench.slice_samples(s, 5.0, 10.0)
    assert [x["t"] for x in out] == [5.0, 9.9]


def test_peak_sample_picks_busiest():
    s = [
        {"cpu_pct": 10.0, "nvidia": {"util": 5.0, "mem_used_mb": 100.0}, "intel": {}},
        {"cpu_pct": 80.0, "nvidia": {"util": 90.0, "mem_used_mb": 200.0}, "intel": {}},
    ]
    assert run_bench.peak_sample(s)["cpu_pct"] == 80.0


def test_peak_sample_empty():
    assert run_bench.peak_sample([]) == {}
