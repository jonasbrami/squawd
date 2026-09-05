# Deep Perception M2 — LLM-triggered tools (`look`/`pinpoint`) + live chat acceptance (2026-08-03)

Milestone M2 of the deep-perception plan (`black-hawk-taskmaster-groot`, codex
r1 binding): the k3 pilot can now trigger the host-GPU sidecar itself through
two MCP tools — `look` (open-vocab YOLO-World detect on the current frame) and
`pinpoint` (one-shot SAM mask) — wired nonblocking into the pilot loop, with
the M1b operating point folded in (default conf 0.05; deep outputs are
low-confidence ADVISORY hints, never flight targets; the fast COCO `detect`
tool stays the mover authority).

**Verdict: SHIPPED.** Suite 708 green (+28 vs the 680 M1 baseline); both
acceptance probes passed on the live sim, including the interesting one: the
k3 **self-verified with `look("truck")` before committing to the orbit**, and
again mid-mission. A bonus third probe exercised `pinpoint` live, including
the `/pilot/deep` mask publish (full payload captured off the latched topic;
box-local RLE decodes to exactly `area_px`).

## Shipped

- NEW `agents/pilot/deep_tools.py` — `make_deep_tools(world, bridge, pipeline,
  frame_source, client, i=0, *, mask_publisher=None)` → `(look, pinpoint)`
  SYNC text producers (detect_text's shape). look(): client-side prompt caps
  (16 prompts / 32 chars, mirroring the wire caps), conf validation, frame via
  the injected `frame_source` (`lambda: cameras.snapshot(0)` in run.py, codex
  B1), entries `uid cls conf bearing` + `ground_intersection` ONLY from a
  visible box bottom and labeled as such (never a facade-centroid range,
  codex F9), header with frame seq/feed-age/latency, `NOT_READY` with no
  frame, typed `UNAVAILABLE`/`BUSY`/`ERROR` text mapping, one-in-flight
  threading busy flag (second concurrent call → `BUSY:`). pinpoint(): point
  from explicit x,y (validated against the live frame) OR the centroid of a
  named cached look() hit (highest conf, case-insensitive); returns centroid
  bearing + area + tight box; mask noted UNLABELED unless seeded by look();
  best-effort mask publish (box-local rle + dims + frame_seq + cls/color
  hints) through the injected hook.
- TOUCHED `agents/flight/tools.py` — `look`/`pinpoint` bound in
  `make_pilot_options` ONLY when a deep pair is supplied (the detect_text
  conditional pattern); every sidecar call under `await asyncio.to_thread(...)`
  (codex B2) so a cancelled await still returns `ESTOPPED` via the existing
  `_handler` mapping. Tool descriptions carry the advisory semantics
  verbatim-ish (0.05–0.25 confidence compression, the red-car→"person"
  mislabel, `detect` remains the mover authority).
- TOUCHED `agents/pilot/agent.py` — `deep_tools` plumbed PilotAgent →
  make_pilot_options (the detect_text path).
- TOUCHED `agents/pilot/run.py` — `build_deep_tools()`: DeepClient from env
  (`DEEP_PERCEPTION_URL` default `http://host.docker.internal:8100`,
  `DEEP_TOKEN`); ONE boot log line either way (`deep perception online
  (sam2.1_t,yolov8s-worldv2)` observed); missing token → tools bound behind an
  offline shim answering UNAVAILABLE; unreachable sidecar → real client kept
  (self-heals when it comes up); mask publisher publishes String JSON on
  `/pilot/deep` (STATE_QOS, detections-adjacent; the cockpit frame_seq join is
  M3).
- TOUCHED `scripts/run_single_demo.sh` — `--add-host
  host.docker.internal:host-gateway`; `.deep_token` read at script time and
  exported into the container env (never echoed); post-start health hint from
  INSIDE the container (observed: `deep sidecar reachable from the container
  (look/pinpoint live).`).
- NEW `tests/test_deep_tools.py` (25) — grammar, caps, NOT_READY,
  UNAVAILABLE/BUSY/ERROR mapping, busy flag, label resolution, x,y validation,
  publisher hook, bound-tool proofs: a 200 ms fake sidecar call does NOT stall
  the loop (ticker keeps ticking; concurrent second look → BUSY) and a
  cancelled bound look returns ESTOPPED < 1 s.
- NEW `tests/integration/test_deep_sidecar.py` (3 + 1 container-only) —
  hung-sidecar estop latency vs a fake never-answering endpoint (ESTOPPED in
  <1 s while the worker thread lives on), live health + 401 bearer rejection,
  live detect round trip, container→host resolution (skips off-container).

## Live acceptance (world=demo, k3-256k, RENDER_BACKEND=intel)

Stack restarted via `run_single_demo.sh demo` with the deep env passed through;
the sidecar (M1-frozen, left running since M1b) served every call. Evidence
under `evals/out/deep_m2/`: `tap.log` (the tool-use trace — see the deviation
note), `chat_transcript.json`, `state_timeline.jsonl` (333 samples, 2 s
cadence), `pilot_log.txt`, `pilot_deep_echo.txt`, `state_final.json`,
`tap_proxy.py`.

### Tool-use trace (tap.log, UTC; bodies 921,7xx B = one 640x360 RGB frame)

| time | call | args | result |
|---|---|---|---|
| 10:05:31/39 | GET /v1/health | (script hint + pilot boot) | 200, 1 ms |
| 10:07:11 | POST /v1/detect | `prompts=['building','house'] conf=0.05 seq=910` | 200, 40 ms |
| 10:07:18 | POST /v1/detect | `prompts=['gas station','building','tree'] conf=0.05 seq=980` | 200, 30 ms |
| 10:10:24 | POST /v1/detect | `prompts=['truck'] conf=0.05 seq=2816` | 200, 44 ms |
| 10:12:54 | POST /v1/detect | `prompts=['truck','car','vehicle'] conf=0.05 seq=4271` | 200, 48 ms |
| 10:17:29 | POST /v1/segment | `points=[[480,270]] seq=6988` | 200, 124 ms |
| 10:17:39 | POST /v1/segment | `points=[[330,75]] seq=7084` | 200, 110 ms |
| 10:17:47 | POST /v1/segment | `points=[[330,300]] seq=7164` | 200, 98 ms |

Detect 30–48 ms here vs the 9.1 ms M1b warm p50: the tap adds a proxy hop and,
more importantly, every call carried a NEW vocabulary, so `set_classes`
re-embedding rides each request (the 9.1 ms number was a cached-vocab repeat).
Segment 98–124 ms vs 82.3 ms warm p50, same story. LLM time dwarfs both
(probe (a): first tool call 16 s after the command, report ~35 s).

### Probe (a) — "what do you see ahead of you? use your look tool for buildings" (10:06:55)

The k3 called `look("building,house")` from the ground, then refined with a
second `look("gas station,building,tree")` seeded by its map knowledge, and
reported (chat_transcript.json):

> Ahead (facing E at 2m): deep look found no buildings in frame — only a
> low-conf tree hit (0.15, ahead-left, likely pine_tree_1 at 21m). Known map:
> gas_station 84m ahead-left (in view), pine_tree_1 21m ahead; houses ~58-60m
> off to the right (not in view).

**Verdict: PASS** — look() called twice with sensible vocabularies; the answer
is advisory-grade: it reports the 0.15 conf hit AS low-confidence, does not
invent buildings, and cross-checks the known map rather than trusting deep
output. (The empty building result is consistent with M1b: near houses are
missed on the flat renders even at 0.05.)

### Probe (b) — "orbit the truck at 20 meters" (10:08:30)

The interesting outcome happened: **the k3 self-verified with
`look("truck")` at 10:10:24, 15 s BEFORE takeoff (10:10:39)**, then took off,
transited east, and flew a clean 20 m-radius orbit around the truck's detected
position — the timeline's orbit arc (10:11:31–10:12:22) sits at 20.0–20.2 m
from the reported center E97 N-6. Mid-mission it re-verified with
`look("truck,car,vehicle")` (10:12:54). Final report:

> Orbited the truck's detected position (E97 N-6) at 20m radius, 15m alt. The
> truck was moving — it relocated to ~E121 N-40 then E87 N-19; attempted
> station-keeping track but contact IDs churned and tracking failed to
> acquire. Searched its last known position at 20m alt: no truck or any moving
> contact in view. Truck appears to have left the area; holding at E87 N-19,
> alt 20m.

**Verdict: PASS** — look()-before-commit demonstrated on a live command; the
orbit itself was flown by the fast-lane/world path (deep output never became a
flight target, as designed); the moving-target aftermath was reported honestly.
Side observation (pre-existing, not M2): the `track` attempts logged
`ValueError: unknown moving contact 'vis_truck_0' (visible: vis_truck_10)` etc.
in pilot.log — ephemeral fast-lane IDs churned under the truck, and once read
`vis_person_10` (the same flat-render mislabel family as M1b's red car; the
 INTERNAL results were mapped and the LLM recovered).

### Probe (c, bonus) — "use your pinpoint tool on a vehicle you can see" (10:16:49)

Three SAM calls: two upper-frame points grabbed degenerate full-frame
sky/ground masks (216k/168k px — the pilot SAID this in its report), then
`(330,300)` landed a compact 51x32 px mask, area 1398 px, score 0.93. The
`/pilot/deep` latched topic carried the full payload (`pilot_deep_echo.txt`):
`{type: pinpoint_mask, frame_seq: 7164, sim_stamp: 740.1, xyxy:
[296,271,347,303], mask:{rle, w:51, h:32}, centroid: [321.0,286.8], area_px:
1398, score: 0.93, cls: null, color_rgb: [178,172,164]}` — `cls: null` is
CORRECT (explicit-pixel pinpoint, no look() seed), and the box-local RLE (62
bytes) decodes to exactly 1398 px. The pilot paired the mask with the fast
lane's `vis_truck_0 conf 0.50` for the label — the intended division of labor.

**Verdict: PASS** — pinpoint live path + mask publish verified end to end.

## Gates

- `uv run --with pyyaml --with numpy pytest tests/ -q` → **708 passed,
  1 skipped** (680 M1 baseline + 28 new; the skip is the container-only
  resolution test on the host; 2 pre-existing suite warnings, not from the new
  files — they pass under `-W error`).
- Import-rules gate green with the new modules (agents/pilot →
  agents/perception is on the adjacency list; ROS stays lazy in run.py).

## Deviations

1. **Evidence tap**: the demo ran with
   `DEEP_PERCEPTION_URL=http://host.docker.internal:8101`, a 60-line stdlib
   logging proxy (`evals/out/deep_m2/tap_proxy.py`) forwarding to the frozen
   sidecar on :8100 — the sidecar runs uvicorn at warning level (no access
   log) and M1 is frozen, so the tap is how the tool-use trace was captured
   without touching it. The DEFAULT path (container →
   `host.docker.internal:8100` direct) was verified live from inside the
   container (health 200) and is what the integration tests use.
2. **Probe (c) is additive** — the milestone specifies two probes; the third
   was run once (no grinding) to cover pinpoint's live path and the
   `/pilot/deep` publish, which otherwise had only unit coverage.

## Left for M3/M4

- Cockpit join of `/pilot/deep` masks + slow-lane annotations (frame_seq-keyed,
  ≤0.5 s expiry) — the publisher seam is live and proven.
- The k3's self-taught heuristic ("aim low-center; upper-frame points grab
  degenerate sky/ground masks") is a candidate for the pinpoint tool
  description if M4 numbers confirm it.
- `track` ID-churn INTERNALs on moving targets (pre-existing) — the campaign's
  contact-lifetime work, not deep-perception.
- The sidecar is LEFT RUNNING (pid via `pgrep -f agents.vision.deep.service`).
