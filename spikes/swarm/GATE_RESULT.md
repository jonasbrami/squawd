# Bridge Spike Gate — RESULT

**Status: ✅ PASS** (2026-06-14)

The §7 highest-risk unknown of the swarm design is retired: a **Claude Agent SDK
in-process tool, on a single asyncio event loop, in one process**, can:
- `await` MAVSDK to arm + take off a PX4 SITL drone, AND
- read live ROS2 telemetry via `rclpy` (the `RosBridge` thread),

against PX4 SITL + Gazebo Harmonic, with no cross-loop / cross-thread errors.

## Evidence (gate_spike.py run)
- `MAVSDK connected to PX4`
- Claude SDK init: `apiKeySource: 'none'` (OAuth via mounted `/root/.claude`), model
  `claude-opus-4-8`, MCP server `flight: connected`.
- Claude called `mcp__flight__takeoff_and_report`; tool returned
  `Took off. ROS2-reported altitude: 5.10 m` (drone actually climbed to ~5 m; altitude
  read through rclpy, not MAVSDK).
- Final: `ResultMessage(subtype='success')`, `GATE: agent run completed`.

## Key findings / fixes discovered during the spike
- **ROS2 Jazzy** (not Humble) — matches Ubuntu 24.04 + Gazebo Harmonic.
- **uv** for Python: `uv venv --system-site-packages` so the venv still sees apt-installed
  `rclpy`/`px4_msgs` (which arrive on PYTHONPATH via `source setup.bash`); run with ROS sourced.
- **mavsdk_server must be version-matched to the pip client.** A standalone v2.12.2 server
  vs pip client v3.15.3 left gRPC 50051 dead and `connect()` hung. Fix: symlink the
  `mavsdk_server` bundled inside the pip `mavsdk` package.
- **PX4 gz_bridge** needs `cppzmq-dev` (CPPZMQ::CPPZMQ CMake target) — gz-harmonic doesn't pull it.
- **OAuth in-container**: install node + `@anthropic-ai/claude-code`, bind-mount creds to
  `/root/.claude`; `/root/.claude.json` must be valid JSON (`{}` minimum — an empty 0-byte
  file crashes the CLI). `setting_sources=[]` keeps the SDK from inheriting host settings/hooks.
- **PX4 sends offboard MAVLink to udp 14540** (binds local 14580); uXRCE-DDS client → ROS2 gives
  ~29-43 `/fmu` topics including `vehicle_local_position`.

## Harmless
- `Exception ignored in <System.__del__>: ImportError: sys.meta_path is None` — MAVSDK cleanup
  during interpreter shutdown, after success. Cosmetic.

## Cost note
- One gate run ≈ $0.36 (Opus, ~15.7k cache-creation tokens even with `setting_sources=[]`).
  Per-drone continuously-reasoning agents will need cost attention for the swarm (see spec §feasibility).
