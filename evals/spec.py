"""TaskSpec: load + validate a task YAML into typed objects.

Validation fails fast (SpecError) on missing fields or unknown oracle checks, so a
sweep never wastes sim time on a malformed scenario."""
from dataclasses import dataclass

import yaml

from evals.oracle import CHECKS


class SpecError(Exception):
    pass


@dataclass(frozen=True)
class SeedObject:
    id: str
    e: float
    n: float


@dataclass(frozen=True)
class SetupSpec:
    world: str
    spawn: str
    seed_objects: list[SeedObject]


@dataclass(frozen=True)
class BudgetSpec:
    wall_clock_s: float
    max_steps: int


@dataclass(frozen=True)
class TaskSpec:
    id: str
    difficulty: dict
    setup: SetupSpec
    prompt: str
    budget: BudgetSpec
    oracle: list[dict]
    suite: str | None = None
    pilot: list[dict] | None = None   # ideal tool sequence for the no-LLM trap gate
    # must-FAIL baseline (dynamic tasks): the naive strategy — usually a tail
    # chase — that the task exists to defeat. If it passes, the rung is a bug.
    null_pilot: list[dict] | None = None

    def objects_map(self) -> dict[str, tuple[float, float]]:
        return {o.id: (o.e, o.n) for o in self.setup.seed_objects}


def _require(d: dict, key: str, ctx: str):
    if not isinstance(d, dict) or key not in d:
        raise SpecError(f"missing '{key}' in {ctx}")
    return d[key]


def load_task(path: str) -> TaskSpec:
    with open(path) as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise SpecError(f"{path}: top level must be a mapping")

    s = _require(raw, "setup", path)
    if not isinstance(s, dict):
        raise SpecError(f"{path}: 'setup' must be a mapping")
    seeds = [SeedObject(id=_require(o, "id", "seed_object"),
                        e=float(_require(o, "east", "seed_object")),
                        n=float(_require(o, "north", "seed_object")))
             for o in s.get("seed_objects", [])]
    setup = SetupSpec(world=_require(s, "world", "setup"),
                      spawn=s.get("spawn", "home"),
                      seed_objects=seeds)

    b = _require(raw, "budget", path)
    if not isinstance(b, dict):
        raise SpecError(f"{path}: 'budget' must be a mapping")
    budget = BudgetSpec(wall_clock_s=float(_require(b, "wall_clock_s", "budget")),
                        max_steps=int(_require(b, "max_steps", "budget")))

    oracle = _require(raw, "oracle", path)
    if not isinstance(oracle, list) or not oracle:
        raise SpecError(f"{path}: 'oracle' must be a non-empty list")
    for chk in oracle:
        name = _require(chk, "check", "oracle entry")
        if name not in CHECKS:
            raise SpecError(f"{path}: unknown oracle check '{name}' "
                            f"(have {sorted(CHECKS)})")

    def _check_script(key: str):
        script = raw.get(key)
        if script is None:
            return None
        if not isinstance(script, list) or not script:
            raise SpecError(f"{path}: '{key}' must be a non-empty list")
        for step in script:
            if "behavior" not in step:
                _require(step, "tool", f"{key} step")
        return script

    pilot = _check_script("pilot")
    null_pilot = _check_script("null_pilot")

    return TaskSpec(
        id=_require(raw, "id", path),
        difficulty=_require(raw, "difficulty", path),
        setup=setup,
        prompt=_require(raw, "prompt", path),
        budget=budget,
        oracle=oracle,
        suite=raw.get("suite"),
        pilot=pilot,
        null_pilot=null_pilot,
    )
