"""Interactive swarm: a Commander agent + N drone agents, all on one event loop.

- You type a command in the observatory -> it lands on /swarm/user_input.
- The Commander agent reads it, sees the drones' positions, and BROADCASTS
  per-drone instructions on /swarm/chat ("commander: drone_0 take off and go ...").
- Each drone agent runs a continuous react loop: when a NEW relevant message
  appears, it acts via its tools (take_off / goto / orbit / look / land / ...).

Scales to N drones (SWARM_N): one persistent Claude client per agent; drones only
spend tokens when a relevant message arrives.
"""
import asyncio
import os
import shutil

from std_msgs.msg import String
from mavsdk import System
from px4_msgs.msg import VehicleLocalPosition
from claude_agent_sdk import ClaudeSDKClient

from agents.core.bus import RosBridge, CHAT_QOS
from agents.core.store import TopicLog
from agents.core.camera import GzCameras
from agents.world import World
from agents.flight import make_drone_options
from agents.swarm.loops import make_commander, commander_loop, drone_loop

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
    chat = TopicLog(bridge, "/swarm/chat", String, CHAT_QOS)
    user = TopicLog(bridge, "/swarm/user_input", String, CHAT_QOS)
    for i in range(N):
        bridge.subscribe(f"/px4_{i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
    bridge.start()

    def publish_chat(text: str) -> None:
        m = String()
        m.data = text
        bridge.publish("/swarm/chat", String, m, CHAT_QOS)

    drones = []
    for i in range(N):
        d = System(mavsdk_server_address="127.0.0.1", port=50051 + i)
        await d.connect()
        async for s in d.core.connection_state():
            if s.is_connected:
                break
        drones.append(d)
        print(f"drone_{i} connected", flush=True)

    # Reuse PX4's own geofence as the hard safety layer (autopilot-enforced even on
    # link loss) rather than custom Python bounds-checks. Warning action so it never
    # disrupts the demo; raise GF_ACTION later to actually contain drones.
    for idx, d in enumerate(drones):
        try:
            await d.param.set_param_float("GF_MAX_HOR_DIST", 300.0)
            await d.param.set_param_float("GF_MAX_VER_DIST", 80.0)
            await d.param.set_param_int("GF_ACTION", 1)
        except Exception as e:
            print(f"geofence setup skipped for drone_{idx}: {e}", flush=True)

    world = World()
    cameras = GzCameras(N)                 # start reading each drone's camera off gz
    print(f"perception: {len(world.buildings)} buildings loaded; "
          f"cameras subscribed for {N} drones.", flush=True)

    commander = make_commander(N, publish_chat, env=agent_env("commander"))
    drone_clients = [
        ClaudeSDKClient(options=make_drone_options(
            i, drones[i], world, bridge, N, cameras, publish_chat, env=agent_env(f"drone{i}")))
        for i in range(N)
    ]
    print(f"swarm online: commander + {N} drones. Waiting for commands on /swarm/user_input.",
          flush=True)
    await asyncio.gather(
        commander_loop(commander, user, world, bridge, N),
        *[drone_loop(i, drone_clients[i], chat) for i in range(N)],
    )


if __name__ == "__main__":
    asyncio.run(main())
