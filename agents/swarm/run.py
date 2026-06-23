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
drones only spend tokens when the Commander tasks them.
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
    user = TopicLog(bridge, "/swarm/user_input", String, CHAT_QOS)
    # Per-drone directed channels: commander -> drone on cmd_logs[i], drone ->
    # commander on report_logs[i]. (Same node publishes and subscribes; rclpy
    # delivers to its own subscriptions, as the chat bus already relied on.)
    # Topic tokens can't start with a digit (ROS 2 rule), so use drone_<i>, not <i>.
    cmd_logs = [TopicLog(bridge, f"/swarm/cmd/drone_{i}", String, CHAT_QOS) for i in range(N)]
    report_logs = [TopicLog(bridge, f"/swarm/report/drone_{i}", String, CHAT_QOS) for i in range(N)]
    for i in range(N):
        bridge.subscribe(f"/px4_{i}/fmu/out/vehicle_local_position", VehicleLocalPosition)
    bridge.start()

    def publish_to(topic: str, text: str) -> None:
        m = String()
        m.data = text
        bridge.publish(topic, String, m, CHAT_QOS)

    def publish_chat(text: str) -> None:        # /swarm/chat is a read-only UI mirror
        publish_to("/swarm/chat", text)

    def dispatch(i: int, task: str) -> None:    # commander -> drone_i (+ mirror)
        publish_to(f"/swarm/cmd/drone_{i}", task)
        publish_chat(f"commander→drone_{i}: {task}")

    def make_report(i: int):                    # drone_i -> commander (+ mirror)
        def report(message: str) -> None:
            publish_to(f"/swarm/report/drone_{i}", message)
            publish_chat(f"drone_{i}: {message}")
        return report

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

    commander = make_commander(N, dispatch, env=agent_env("commander"))
    drone_clients = [
        ClaudeSDKClient(options=make_drone_options(
            i, drones[i], world, bridge, N, cameras, make_report(i), env=agent_env(f"drone{i}")))
        for i in range(N)
    ]
    print(f"swarm online: commander + {N} drones. Waiting for commands on /swarm/user_input.",
          flush=True)
    await asyncio.gather(
        commander_loop(commander, user, report_logs, world, bridge, N),
        *[drone_loop(i, drone_clients[i], cmd_logs[i]) for i in range(N)],
    )


if __name__ == "__main__":
    asyncio.run(main())
