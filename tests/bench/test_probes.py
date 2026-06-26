from bench import probes


def test_parse_nvidia_smi():
    d = probes.parse_nvidia_smi("42, 1536, 78.50")
    assert d == {"util": 42.0, "mem_used_mb": 1536.0, "power_w": 78.5}


def test_parse_intel_gpu_top_matches_render_and_video():
    obj = {"engines": {
        "Render/3D": {"busy": 65.4, "unit": "%"},
        "Blitter/0": {"busy": 0.0, "unit": "%"},
        "Video/0": {"busy": 12.0, "unit": "%"},
        "VideoEnhance/0": {"busy": 0.0, "unit": "%"},
    }}
    d = probes.parse_intel_gpu_top(obj)
    assert d["render_pct"] == 65.4
    assert d["video_pct"] == 12.0


def test_parse_docker_stats_gib():
    d = probes.parse_docker_stats("231.40% 4.5GiB / 31GiB")
    assert d["cpu_pct"] == 231.4
    assert round(d["mem_mb"], 1) == 4608.0


def test_parse_docker_stats_mib():
    d = probes.parse_docker_stats("12.00% 800MiB / 31GiB")
    assert round(d["mem_mb"], 1) == 800.0


def test_parse_gz_rtf():
    text = "real_time_factor: 0.984\nsim_time {\n  sec: 12\n}\n"
    assert probes.parse_gz_rtf(text) == 0.984


def test_limiting_resource_picks_max_normalized():
    sample = {"cpu_pct": 50.0, "ram_used_gb": 6.0,
              "nvidia": {"util": 95.0, "mem_used_mb": 2000.0, "power_w": 90.0},
              "intel": {"render_pct": 30.0, "video_pct": 10.0}}
    assert probes.limiting_resource(sample) == "dgpu"


def test_limiting_resource_vram_beats_util():
    sample = {"cpu_pct": 10.0, "ram_used_gb": 2.0,
              "nvidia": {"util": 40.0, "mem_used_mb": 7800.0, "power_w": 50.0},
              "intel": {}}
    assert probes.limiting_resource(sample) == "vram"


def test_limiting_resource_none_when_all_idle():
    sample = {"cpu_pct": 5.0, "ram_used_gb": 1.0,
              "nvidia": {"util": 3.0, "mem_used_mb": 200.0, "power_w": 10.0},
              "intel": {"render_pct": 2.0, "video_pct": 0.0}}
    assert probes.limiting_resource(sample) == "none"
