# src/dronebot/stack.py
"""Shared wiring for the dronebot stack — used by both the terminal REPL
(app.py) and the web server. One construction + lifecycle path (DRY).
"""
from __future__ import annotations

import asyncio
import os
from dataclasses import dataclass, field

from mavsdk import System

from dronebot.agent.claude_agent import DroneAgent
from dronebot.config import Config
from dronebot.control.controller import DroneController
from dronebot.control.executor import CommandExecutor
from dronebot.control.safety import SafetyGuard
from dronebot.control.state import StateStore
from dronebot.control.telemetry import start_telemetry
from dronebot.flight_log import FlightLog
from dronebot.perception.gazebo_perception import GazeboPerception
from dronebot.perception.store import PerceptionStore

_RGB_TOPIC = os.environ.get("DRONEBOT_RGB_TOPIC", "/camera")
_DEPTH_TOPIC = os.environ.get("DRONEBOT_DEPTH_TOPIC", "/depth_camera")


@dataclass
class Stack:
    config: Config
    drone: System
    controller: DroneController
    state: StateStore
    perception_store: PerceptionStore
    perception: GazeboPerception
    executor: CommandExecutor
    agent: DroneAgent
    log: FlightLog
    telemetry_tasks: list = field(default_factory=list)


def build_stack(config: Config) -> Stack:
    drone = System()
    controller = DroneController(drone)
    state = StateStore()
    perception_store = PerceptionStore()
    perception = GazeboPerception(perception_store, _RGB_TOPIC, _DEPTH_TOPIC)
    guard = SafetyGuard(config.limits)
    executor = CommandExecutor(controller, state, guard)
    agent = DroneAgent(executor, perception_store, config.model)
    log_dir = os.environ.get("DRONEBOT_LOG_DIR", "flight_logs")
    os.makedirs(log_dir, exist_ok=True)
    log = FlightLog(os.path.join(log_dir, "session.jsonl"))
    return Stack(config, drone, controller, state, perception_store,
                 perception, executor, agent, log)


async def start_stack(stack: Stack) -> None:
    await stack.controller.connect(stack.config.connection_url)
    stack.state.set_connection(True)
    stack.telemetry_tasks = start_telemetry(stack.drone, stack.state)
    for _ in range(40):  # wait for a position fix, then set home
        if stack.state.position is not None:
            stack.state.set_home(stack.state.position)
            break
        await asyncio.sleep(0.25)
    await stack.perception.start()


async def stop_stack(stack: Stack) -> None:
    await stack.perception.stop()
    for task in stack.telemetry_tasks:
        task.cancel()
