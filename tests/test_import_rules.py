"""The architecture gate (ICD §0.1): AST scan of every agents/** file against
the normative adjacency list. Any forbidden agents.*, ROS, gz, or numpy import
fails the build — this is the anti-spaghetti law, executable.
"""
import ast
from pathlib import Path

# Normative adjacency list (kept in sync with ICD §0.1). Self-imports within a
# package are always allowed and therefore included implicitly.
IMPORT_RULES = {
    "agents":             {"agents.core"},   # the namespace root itself
    "agents/core":        {"agents.core"},
    "agents/world":       {"agents.world"},
    "agents/perception":  {"agents.perception"},
    "agents/vision":      {"agents.core", "agents.world", "agents.perception",
                           "agents.vision"},
    "agents/flight":      {"agents.core", "agents.world", "agents.perception",
                           "agents.flight"},
    "agents/pilot":       {"agents.core", "agents.world", "agents.perception",
                           "agents.vision", "agents.flight", "agents.pilot",
                           "agents.observatory"},
    "agents/observatory": {"agents.core", "agents.observatory"},
}

# ROS / gz / numpy gates (third-party libs are governed by requirements pinning).
ROS_ROOTS = ("rclpy", "std_msgs", "px4_msgs")
GZ_ROOTS = ("gz",)
ROS_ALLOWED = {"agents/core", "agents/pilot", "agents/observatory"}
GZ_ALLOWED = {"agents/core"}
NUMPY_ALLOWED = {"agents/vision"}

AGENTS = Path("agents")


def _imports_of(path: Path) -> set[str]:
    """Module-SCOPE import roots (top-level statements only). Lazy in-function
    imports are the sanctioned runtime-injection pattern (estop/gzposes/
    pipeline), so the ROS/numpy gates apply only at module scope."""
    tree = ast.parse(path.read_text(), filename=str(path))
    roots = set()
    for node in tree.body:                     # top-level statements ONLY
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def _agents_imports_of(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name for a in node.names if a.name.startswith("agents."))
        elif isinstance(node, ast.ImportFrom) and node.module \
                and node.module.startswith("agents."):
            mods.add(node.module)
    return mods


def test_import_rules():
    violations = []
    for path in sorted(AGENTS.rglob("*.py")):
        parts = path.parts
        pkg = "/".join(parts[:2]) if len(parts) > 2 else "agents"
        if pkg not in IMPORT_RULES:
            violations.append(f"{path}: package {pkg} is not in the adjacency list")
            continue
        allowed = IMPORT_RULES[pkg]

        for mod in _agents_imports_of(path):
            top = ".".join(mod.split(".")[:2])
            if top not in allowed:
                violations.append(f"{path}: forbidden import {mod} (allowed: {sorted(allowed)})")

        roots = _imports_of(path)
        if pkg not in ROS_ALLOWED:
            for r in ROS_ROOTS:
                if r in roots:
                    violations.append(f"{path}: forbidden ROS import {r}")
        if pkg not in GZ_ALLOWED and any(g in roots for g in GZ_ROOTS):
            violations.append(f"{path}: forbidden gz import")
        if pkg not in NUMPY_ALLOWED and "numpy" in roots:
            violations.append(f"{path}: numpy is confined to agents/vision")

    assert not violations, "dependency-law violations:\n" + "\n".join(violations)


def test_adjacency_list_covers_existing_packages():
    pkgs = {"/".join(p.parts[:2]) if len(p.parts) > 2 else "agents"
            for p in AGENTS.rglob("*.py")}
    missing = pkgs - set(IMPORT_RULES)
    assert not missing, f"packages missing from IMPORT_RULES: {missing}"
