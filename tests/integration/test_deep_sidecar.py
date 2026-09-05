"""M2 integration: the live host sidecar + the container seam.

Skip-clean rules: the live-service tests probe `.deep_token` + the docker
gateway health at collection and skip when the sidecar is down; the
container-resolution test additionally needs to run INSIDE pilot-sim. The
hung-sidecar estop-latency test uses its own fake endpoint and runs anywhere.
"""
import asyncio
import http.server
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from agents.flight import FlightOps, make_pilot_options
from agents.perception.deep_client import DeepClient
from agents.pilot.deep_tools import make_deep_tools

ROOT = Path(__file__).resolve().parents[2]
TOKEN_FILE = ROOT / ".deep_token"
GW_URL = os.environ.get("DEEP_PERCEPTION_URL", "http://172.17.0.1:8100")


def _token() -> str:
    return TOKEN_FILE.read_text().strip()


def _service_up() -> bool:
    if not TOKEN_FILE.exists():
        return False
    try:
        return DeepClient(base_url=GW_URL, token=_token()).health().ok
    except Exception:
        return False


SERVICE = _service_up()
needs_service = pytest.mark.skipif(not SERVICE,
                                   reason="deep sidecar/token not available")


class _World:
    def drone_state(self, bridge, i):
        return None

    def attitude_at(self, t):
        return None


def _frame(w=64, h=64):
    return SimpleNamespace(seq=1, sim_stamp=0.0, width=w, height=h,
                           rgb=bytes(w * h * 3))


# ---------- hung sidecar vs tool cancellation (fake endpoint, no GPU) ----------

def test_hung_sidecar_estop_latency():
    """A sidecar stuck in read must not hold the pilot loop: the bound look
    tool's await returns ESTOPPED promptly on cancellation (the worker thread
    lives on, bounded by the client timeouts — here released by the test)."""
    release = threading.Event()

    class HungHandler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            release.wait(timeout=15)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b"{}")

        def log_message(self, *a):
            pass

    srv = http.server.HTTPServer(("127.0.0.1", 0), HungHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    port = srv.server_address[1]
    client = DeepClient(base_url=f"http://127.0.0.1:{port}", token="t",
                        read_timeout_detect=30.0)
    look, pinpoint = make_deep_tools(_World(), None, None, _frame, client)
    opts = make_pilot_options(FlightOps(None, None, None, 0, 1),
                              deep_tools=(look, pinpoint),
                              report=lambda m: None)
    instance = opts.mcp_servers["pilot"]["instance"]

    import mcp.types as mcp_types

    async def go():
        req = mcp_types.CallToolRequest(
            method="tools/call",
            params=mcp_types.CallToolRequestParams(name="look",
                                                   arguments={"what": "house"}))
        task = asyncio.create_task(
            instance.request_handlers[mcp_types.CallToolRequest](req))
        await asyncio.sleep(0.4)             # the call is stuck in read
        t0 = time.monotonic()
        task.cancel()
        res = (await task).root
        return res, time.monotonic() - t0

    try:
        res, dt = asyncio.run(go())
    finally:
        release.set()
        srv.server_close()
    assert res.isError
    assert res.content[0].text.startswith("ESTOPPED: operator halted look")
    assert dt < 1.0, f"estop latency vs hung sidecar: {dt:.2f}s"


# ---------- live sidecar (skip unless the service is up) ----------

@needs_service
def test_live_health_and_bearer_rejection():
    res = DeepClient(base_url=GW_URL, token=_token()).health()
    assert res.ok
    assert set(res.data["models_loaded"]) >= {"yolov8s-worldv2", "sam2.1_t"}
    bad = DeepClient(base_url=GW_URL, token="wrong").health()
    assert not bad.ok and "401" in bad.detail


@needs_service
def test_live_detect_round_trip_on_a_synthetic_frame():
    """The tool path end to end against the warm service (a black frame
    yields zero or more dets — the point is the wire, not the content)."""
    client = DeepClient(base_url=GW_URL, token=_token())
    look, _ = make_deep_tools(_World(), None, None,
                              lambda: _frame(640, 360), client)
    text = look("building,car")
    assert "advisory deep hit(s) for 'building,car'" in text
    assert "frame #1" in text


# ---------- container → host name resolution (skip outside a container) ----------

@needs_service
@pytest.mark.skipif(not Path("/.dockerenv").exists(),
                    reason="not inside a container")
def test_container_resolves_host_gateway():
    url = os.environ.get("DEEP_PERCEPTION_URL",
                         "http://host.docker.internal:8100")
    token = os.environ.get("DEEP_TOKEN") or _token()
    import socket
    assert socket.gethostbyname("host.docker.internal")
    assert DeepClient(base_url=url, token=token).health().ok
