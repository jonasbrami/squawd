"""Doctor gate (design §13 item 1): the preflight script exists, is executable,
fails loudly on a machine without sim deps (like this one), and the launcher
refuses to start the pilot when it fails."""
import os
import stat
import subprocess


def test_doctor_script_exists_executable_and_fails_without_deps():
    path = "scripts/doctor_sim.sh"
    assert os.path.isfile(path)
    assert os.stat(path).st_mode & stat.S_IXUSR
    # On a host without the sim runtime (no rclpy/px4_msgs/gz), doctor must FAIL
    # (exit non-zero) with legible output — fail closed, never silent.
    res = subprocess.run(["bash", path], capture_output=True, text=True,
                         timeout=120)
    assert res.returncode != 0
    assert "FAIL" in res.stdout


def test_launcher_refuses_to_start_when_doctor_fails():
    src = open("scripts/run_single_demo.sh").read()
    assert "doctor_sim.sh" in src
    assert "refusing to start the pilot" in src
    assert "exit 1" in src


def test_launcher_copies_only_codex_auth_into_isolated_runtime_home():
    src = open("scripts/run_single_demo.sh").read()
    assert 'cp "$HOME/.codex/auth.json" "$CODEX_CRED/auth.json"' in src
    assert '"$CODEX_CRED:/root/.codex"' in src
    assert "CODEX_HOME=/root/.codex" in src
    assert ".codex/config.toml" not in src
    assert ".codex/plugins" not in src
    assert "cp -r" not in src and "cp -R" not in src
