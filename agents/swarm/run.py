"""Interactive swarm: a Commander agent + N drone agents (a distributed hub).

- You type a command in the observatory -> it lands on /swarm/user_input.
- The Commander reads it, sees the drones' positions, and DISPATCHES a directed
  task to each relevant drone on /swarm/cmd/drone_<i>.
- Each drone is its own agent: it acts on its own /swarm/cmd/drone_<i> with its
  tools, then reports the result back to the Commander on /swarm/report/drone_<i>.
- The Commander mirrors every dispatch and report onto /swarm/chat purely so the
  observatory feed stays legible; the drones do not listen to /swarm/chat.

The agents talk ONLY over ROS topics, so a drone can run on separate hardware
unchanged. Scales to N drones (SWARM_N): one persistent Claude client per agent;
drones only spend tokens when the Commander tasks them. This module is just the
assembler — the agents themselves live in commander.py / drone.py.
"""
import asyncio
import os
import shutil

from agents.core.bus import RosBridge
from agents.core.camera import GzCameras
from agents.core.singleton import acquire_singleton_lock
from agents.world import World
from agents.swarm.commander import CommanderAgent
from agents.swarm.drone import DroneAgent

N = int(os.environ.get("SWARM_N", "3"))


def agent_env(tag: str) -> dict:
    """Isolate each Claude CLI's config so N+1 concurrent clients don't race on one
    file. The `claude` CLI does a non-atomic read-modify-write of $CLAUDE_CONFIG_DIR/
    .claude.json on startup; with many clients booting at once the shared
    /root/.claude.json gets truncated mid-write (JSON EOF) and the whole swarm dies.
    Point each agent at its own dir seeded with a copy of the real credentials.
    Returns the env dict for ClaudeAgentOptions (merged over os.environ)."""
    base = os.environ.get("CLAUDE_CONFIG_DIR", "/root/.claude")
    d = f"/root/.claude-{tag}"
    os.makedirs(d, exist_ok=True)
    try:
        shutil.copy(os.path.join(base, ".credentials.json"), os.path.join(d, ".credentials.json"))
    except Exception:
        pass
    return {"CLAUDE_CONFIG_DIR": d}


async def main():
    bridge = RosBridge(node_name="swarm_agents")
    world = World()
    cameras = GzCameras(N)                 # start reading each drone's camera off gz

    # Construct the agents BEFORE bridge.start(): their __init__ does every ROS
    # subscription (cmd/report/user TopicLogs + per-drone PX4 telemetry), and we
    # want them all registered before the rclpy spin thread comes up.
    commander = CommanderAgent(N, bridge, world, env=agent_env("commander"))
    drones = [DroneAgent(i, world, bridge, N, cameras, env=agent_env(f"drone{i}"))
              for i in range(N)]
    bridge.start()
    print(f"perception: {len(world.buildings)} buildings loaded; "
          f"cameras subscribed for {N} drones.", flush=True)

    for d in drones:                       # connect + geofence each drone's MAVSDK link
        await d.connect()

    print(f"swarm online: commander + {N} drones. Waiting for commands on /swarm/user_input.",
          flush=True)
    await asyncio.gather(commander.run(), *[d.run() for d in drones])


if __name__ == "__main__":
    # Refuse to start a second agents process (stacked Commanders -> dispatch storm).
    # Held for the process lifetime; released automatically on exit.
    _swarm_lock = acquire_singleton_lock()
    asyncio.run(main())
