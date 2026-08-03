# Review findings

The findings below are based on the static working-tree snapshot described in
[README.md](README.md). Severity reflects the current simulation product, while
also considering that flight/control code may later be reused outside SITL.

## Summary

| ID | Severity | Finding |
|---|---|---|
| R1 | High | `goto`, `fly`, and `orbit` do not invoke their existing safety-envelope validators |
| R2 | High | `run_mission` executes unrestricted model-authored Python and bypasses the software envelope |
| R3 | High | The advertised swarm launcher invokes deleted modules |
| R4 | High | A clean clone cannot satisfy the documented build/run path because PX4 is ignored and not provisioned |
| R5 | High | Cockpit command and estop APIs are unauthenticated and exposed on all interfaces |
| R6 | Medium | The single-drone launcher does not start the browser cockpit it publishes a port for |
| R7 | Medium | CPU rendering is incompatible with the single-drone launcher's hard camera preflight |
| R8 | Medium | Dependency manifests do not describe the direct runtime and evaluation dependencies |
| R9 | Medium | Current and historical documentation contradict each other and the source tree |
| R10 | Medium | PX4 setup mutates and clears state inside a large host checkout |
| R11 | Low | Error/degraded paths can obscure safety or readiness failures |
| R12 | Low | Repository data and naming retain substantial obsolete swarm-era surface area |

## R1 — Movement envelope checks are disconnected

Severity: **High**

[`agents/flight/envelope.py`](../agents/flight/envelope.py) implements
`check_goto`, `check_fly_endpoint`, and `check_orbit`, and unit tests exercise
those functions. However, the corresponding handlers in
[`agents/flight/tools.py`](../agents/flight/tools.py) call `FlightOps.goto`,
`FlightOps.fly`, and `FlightOps.orbit` directly. Those methods do not invoke the
validators either. Only takeoff, speed, and explicit track altitude are checked
at the tool boundary in the observed snapshot.

Impact:

- an LLM can issue a normal `goto`, `fly`, or `orbit` outside the advertised
  radius or altitude envelope;
- orbit perimeter validation and endpoint validation are effectively dead code;
- the isolated envelope tests create confidence without proving integration;
- PX4's hard geofence is the remaining protection, but geofence parameter setup
  is allowed to fail and continue in `PilotAgent.connect()`.

Recommended correction:

- resolve the actual endpoint/altitude in each wrapper or centralize validation
  inside `FlightOps` after target resolution;
- integration-test every exposed movement tool against an injected `Envelope`;
- fail startup or visibly enter a restricted mode if PX4 geofence setup fails.

## R2 — `run_mission` is unrestricted code execution in the pilot

Severity: **High**

[`FlightOps.run_mission`](../agents/flight/ops.py) compiles and executes Python
provided by the model using `exec`. The namespace adds useful bindings but does
not restrict Python builtins, imports, filesystem access, subprocesses, network
access, or access to the injected live MAVSDK system. The system prompt also
states that this path is outside the static safety envelope and relies only on
PX4's geofence.

Impact:

- prompt injection or model error can execute arbitrary code with the pilot
  process/container privileges;
- arbitrary MAVSDK actions can bypass fixed-tool validation;
- code can read mounted credentials and repository data;
- code can interfere with the pilot, simulator, or host-mounted workspace.

Recommended correction:

- remove this tool from production-like runs, or replace it with a declarative,
  validated mission plan schema;
- if experimentation requires authored code, isolate it in a separate locked-
  down process/container with no credentials, a narrow RPC, resource limits,
  and an independent safety arbiter;
- never treat Python namespace shaping as a security sandbox.

## R3 — The advertised swarm path is not runnable

Severity: **High**

[`scripts/run_swarm_demo.sh`](../scripts/run_swarm_demo.sh) invokes
`agents/swarm/run.py`. That file does not exist, nor does the Commander module
described by the top-level README and architecture document. Only the legacy
`DroneAgent` remains under `agents/swarm/`.

Impact:

- the primary README quickstart leads users to a startup that cannot launch its
  agent layer;
- the product name and most prominent diagram describe unavailable behavior;
- maintainers can review or modify the wrong architecture.

Recommended correction:

- make the root README explicitly describe the single-drone rebuild;
- remove or clearly archive the broken launcher and legacy class;
- restore Commander/swarm code only when it has an executable assembly and
  current tests.

## R4 — The build is not reproducible from a clean clone

Severity: **High**

The launch scripts require `/workspace/PX4-Autopilot` and specifically an
already-built `build/px4_sitl_default/bin/px4`. The top-level `.gitignore`
excludes `PX4-Autopilot/`; it is not a submodule. The Dockerfile installs Gazebo,
ROS, `px4_msgs`, and application dependencies but does not fetch or build PX4.
This contradicts the README statement that the Docker build compiles PX4.

Impact:

- a fresh checkout plus the documented `docker build` cannot run the project;
- the local ignored checkout is large and contains unreproducible build/state;
- PX4 version, patches, configuration, and build flags are not pinned by this
  repository.

Recommended correction:

- either build a pinned PX4 revision into the image, or provide an explicit
  bootstrap script plus revision/patch manifest;
- fail early with a precise prerequisite message;
- update the quickstart to reflect the real ownership of the PX4 artifact.

## R5 — The cockpit exposes flight control without authentication

Severity: **High**

[`agents/observatory/server.py`](../agents/observatory/server.py) binds uvicorn
to `0.0.0.0:8000`. `/command` and `/estop` accept requests without
authentication, authorization, origin checking, CSRF protection, or an
application-level session. The Docker launch publishes port 8000 on all host
interfaces by default.

Impact:

- any party able to reach the port can submit model commands or trigger land/
  hold;
- camera, detections, state, and chat are also exposed;
- the risk grows materially if this moves beyond a private SITL workstation.

Recommended correction:

- bind the published port to `127.0.0.1` by default;
- add authentication and authorization before any shared-network or real-
  vehicle use;
- enforce origin/CSRF protections and document the interface as unsafe for
  untrusted networks until then.

## R6 — The single-drone launcher omits the cockpit process

Severity: **Medium**

[`scripts/run_single_demo.sh`](../scripts/run_single_demo.sh) publishes port
8000 but starts only the simulator and `agents/pilot/run.py`. It never starts
`agents/observatory/server.py`. The user instructions at the end of the script
therefore offer ROS command publication rather than the implemented cockpit.

Impact: the browser surface is unavailable through the main current launcher,
and the port mapping suggests otherwise.

Recommended correction: start cockpit and pilot as supervised processes, check
both for readiness, and print both log locations; alternatively remove port
8000 and document cockpit startup as a mandatory separate step.

## R7 — CPU mode fails the camera hard gate

Severity: **Medium**

The single-drone launcher advertises `RENDER_BACKEND=cpu`, but that branch uses
`PX4_MODEL=gz_x500`, which has no configured IMX214 camera. The same launcher
then runs `doctor_sim.sh`, whose missing camera topic is a hard failure.

Impact: CPU mode cannot reach pilot startup through the advertised script even
though perception is designed to support a degraded mode.

Recommended correction: choose one explicit policy:

- use the camera-equipped model with software rendering;
- make camera absence a warning and start sensing-degraded; or
- reject CPU mode immediately with a clear unsupported-mode message.

## R8 — Dependency ownership is incomplete

Severity: **Medium**

The code directly imports packages not declared as direct dependencies in
`pyproject.toml`, including NumPy, PyYAML, Starlette, uvicorn, websockets, and
ONNX Runtime for the default production detector. `requirements-swarm.txt`
adds websockets and ONNX Runtime but still relies on transitive dependencies for
other imports. ROS/Gazebo Python modules are system-provided, but that split is
not represented as install metadata or an environment contract.

Impact:

- editable/local installs do not provide all importable modules;
- upgrades of unrelated packages can remove or incompatibly change a
  transitive dependency;
- “works in the container,” “works under uv,” and “tests locally” describe
  different environments.

Recommended correction:

- define explicit extras such as `vision`, `cockpit`, `eval`, and `dev`;
- list every directly imported PyPI package in the appropriate extra;
- keep ROS/Gazebo packages in a documented system-dependency section;
- use the lockfile consistently in image construction rather than separately
  resolving an open-ended requirements file.

## R9 — Documentation and status assertions conflict

Severity: **Medium**

The root README and `docs/architecture.md` describe the removed Commander
system. `docs/PROJECT-STATE.md` describes the single-drone rebuild and is much
closer to the source. Counts and milestone statements also vary within the
living documentation as the worktree evolves.

Impact: new contributors cannot tell which entrypoints, packages, interfaces,
benchmarks, and limitations are authoritative.

Recommended correction:

- designate one current architecture and one current operations guide;
- move historical swarm documents under an explicitly archived heading;
- generate simple facts such as exposed tool names and test/task counts where
  practical;
- date benchmark evidence separately from current implementation status.

## R10 — Simulation startup mutates the host PX4 checkout

Severity: **Medium**

The repository is bind-mounted into the container. `swarm_sim.sh` edits the PX4
OakD model with `sed -i` and removes persisted PX4 parameter/dataman/eeprom state
from the checkout before each launch.

Impact:

- startup is not observational or hermetic;
- concurrent runs or development in the PX4 tree can interfere;
- camera settings may depend on what a prior launch already changed;
- troubleshooting state is destroyed by the next launch.

Recommended correction: copy generated/modified PX4 assets into a run-specific
temporary overlay and place SITL rootfs/state in an explicit disposable volume.
Keep source inputs immutable.

## R11 — Degraded paths hide important readiness failures

Severity: **Low**

Several broad exception handlers intentionally keep the system alive: vision
boot failures yield sensing-degraded, PX4 geofence parameter failures print and
continue, some parameter changes are best-effort, and camera/disconnect loops
swallow exceptions. Graceful degradation is appropriate for noncritical
sensors, but not every failure is equally safe.

Recommended correction:

- classify failures as fatal, safety-degraded, sensing-degraded, or optional;
- publish structured health state rather than relying mainly on logs;
- make safety-degraded status impossible to miss in the cockpit and command
  responses;
- catch narrower exception types where recovery behavior differs.

## R12 — Historical assets obscure the active product boundary

Severity: **Low**

The repository retains swarm naming, hundreds of tracked evaluation-output
files, old launch paths, large experiment documentation, and a legacy swarm
agent next to the rebuild. Preserving research evidence is valuable, but the
active code and archive are not clearly separated.

Recommended correction:

- add an explicit `archive/` or historical-results index;
- keep canonical small summaries tracked and store bulky raw artifacts in a
  release/artifact store with checksums;
- rename active scripts and image files after the single-drone architecture is
  stable, or clearly state that “swarm” names are compatibility names.

## Positive findings worth preserving

- The backend normalization seam prevents SDK message types from leaking into
  the pilot and eval runner.
- `ContactProvider`/`TargetDesignator` protocols separate production vision
  from explicit truth-fed baselines.
- Perception inference, contact fusion, LLM reasoning, and high-rate pursuit are
  separate loops with appropriate ownership.
- Estop uses the same `FlightOps` object as normal tools and explicitly cancels
  the active operation before issuing emergency action.
- Evaluation grading is simulator-state-based rather than an LLM judge, and
  records tool-level transcripts and uncertainty-aware aggregates.
- Coordinate conversions and timestamped pose/attitude interpolation are
  centralized instead of being duplicated across perception and flight code.

