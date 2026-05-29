# src/dronebot/app.py
"""Entrypoint. Owns the single asyncio loop and wires all layers."""
from __future__ import annotations

import asyncio
import os

from dotenv import load_dotenv
from mavsdk import System

from dronebot.agent.claude_agent import DroneAgent
from dronebot.chat.repl import run_repl
from dronebot.config import load_config
from dronebot.control.controller import DroneController
from dronebot.control.executor import CommandExecutor
from dronebot.control.safety import SafetyGuard
from dronebot.control.state import StateStore
from dronebot.control.telemetry import start_telemetry
from dronebot.flight_log import FlightLog
from dronebot.perception.gazebo_perception import GazeboPerception
from dronebot.perception.store import PerceptionStore

# Topic names confirmed via `gz topic -l` in Task 13.
_RGB_TOPIC = os.environ.get("DRONEBOT_RGB_TOPIC", "/camera")
_DEPTH_TOPIC = os.environ.get("DRONEBOT_DEPTH_TOPIC", "/depth_camera")


async def main() -> None:
    load_dotenv()
    cfg = load_config()

    drone = System()
    controller = DroneController(drone)
    print(f"connecting to {cfg.connection_url} ...")
    await controller.connect(cfg.connection_url)
    print("connected.")

    state = StateStore()
    state.set_connection(True)
    telemetry_tasks = start_telemetry(drone, state)

    # Capture home once a position fix arrives.
    for _ in range(40):
        if state.position is not None:
            state.set_home(state.position)
            break
        await asyncio.sleep(0.25)

    perception_store = PerceptionStore()
    perception = GazeboPerception(perception_store, _RGB_TOPIC, _DEPTH_TOPIC)
    await perception.start()

    guard = SafetyGuard(cfg.limits)
    executor = CommandExecutor(controller, state, guard)
    log = FlightLog("flight_logs/session.jsonl")
    os.makedirs("flight_logs", exist_ok=True)

    try:
        async with DroneAgent(executor, perception_store, cfg.model) as agent:
            await run_repl(agent, executor, log)
    finally:
        await perception.stop()
        for task in telemetry_tasks:
            task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
