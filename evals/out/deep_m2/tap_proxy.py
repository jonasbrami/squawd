#!/usr/bin/env python3
"""M2 acceptance evidence tap: a logging reverse proxy on the docker0
gateway (:8101 -> :8100) so the pilot's deep tool calls leave a trace
WITHOUT touching the frozen M1 sidecar (no access logs at warning level).
Each line: ISO time, client, method path, byte count, /v1/detect prompts,
response status, upstream latency ms. stdlib only."""
import http.client
import json
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UPSTREAM = ("172.17.0.1", 8100)
BIND = ("172.17.0.1", 8101)


class Tap(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _proxy(self):
        t0 = time.monotonic()
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else None
        extra = ""
        if self.path == "/v1/detect" and body:
            try:
                d = json.loads(body)
                extra = (f" prompts={d.get('prompts')} conf={d.get('conf')}"
                         f" seq={d.get('frame', {}).get('seq')}")
            except Exception:
                pass
        elif self.path == "/v1/segment" and body:
            try:
                d = json.loads(body)
                extra = (f" points={d.get('points')} box={d.get('box')}"
                         f" seq={d.get('frame', {}).get('seq')}")
            except Exception:
                pass
        conn = http.client.HTTPConnection(*UPSTREAM, timeout=30)
        try:
            conn.request(self.command, self.path, body=body,
                         headers={"Authorization":
                                  self.headers.get("Authorization", ""),
                                  "Content-Type": "application/json"})
            resp = conn.getresponse()
            data = resp.read()
            ms = (time.monotonic() - t0) * 1000
            print(f"{datetime.now(timezone.utc).isoformat(timespec='milliseconds')} "
                  f"{self.client_address[0]} {self.command} {self.path} "
                  f"{length}B{extra} -> {resp.status} {ms:.0f}ms", flush=True)
            self.send_response(resp.status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except Exception as e:
            print(f"{datetime.now(timezone.utc).isoformat(timespec='milliseconds')} "
                  f"{self.client_address[0]} {self.command} {self.path} "
                  f"PROXY_ERROR {e}", flush=True)
            self.send_response(502)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"{}")
        finally:
            conn.close()

    do_GET = _proxy
    do_POST = _proxy

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    print(f"[tap] {BIND[0]}:{BIND[1]} -> {UPSTREAM[0]}:{UPSTREAM[1]}",
          flush=True)
    ThreadingHTTPServer(BIND, Tap).serve_forever()
