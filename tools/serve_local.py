#!/usr/bin/env python3
"""Run the real dashboard + real run logic on a desktop, no Pico needed.

Reuses runner.py and index.html unchanged, so this exercises the actual
contract - not a mock of it. The only thing missing is the motors.

    python3 tools/serve_local.py        # then open http://localhost:8080/
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
PICO = os.path.join(HERE, "..", "pico")
sys.path.insert(0, PICO)

from runner import Runner                       # noqa: E402
from config import GRID                         # noqa: E402

PORT = int(os.environ.get("PORT", "8080"))
STEP_PAUSE_S = 0.7

RUN = Runner()


def robot_thread():
    """Stands in for robot_task(): advance one cell every STEP_PAUSE_S."""
    while True:
        if RUN.state["running"]:
            RUN.advance()
            time.sleep(STEP_PAUSE_S)
        else:
            time.sleep(0.1)


def parse_cell(text):
    try:
        r, c = [int(x) for x in text.split(",")]
    except (ValueError, AttributeError):
        return None
    return (r, c) if 0 <= r < GRID and 0 <= c < GRID else None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj))

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            with open(os.path.join(PICO, "index.html"), "rb") as f:
                self._send(200, "text/html; charset=utf-8", f.read())
        elif u.path == "/state":
            self._json(RUN.state)
        else:
            self._send(404, "text/plain", "not found")

    def do_POST(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == "/select":
            ok, why = RUN.select(q.get("algo", ""))
            self._json({"ok": ok, "algo": RUN.state["algo"],
                        "error": None if ok else why}, 200 if ok else 409)
        elif u.path == "/run":
            start = parse_cell(q["start"]) if "start" in q else None
            ok, why = RUN.run(start=start, heading=q.get("heading"))
            self._json({"ok": ok, "algo": RUN.state["active_algo"],
                        "error": None if ok else why}, 200 if ok else 409)
        elif u.path == "/reset":
            RUN.stop()
            self._json({"ok": True})
        else:
            self._send(404, "text/plain", "not found")


if __name__ == "__main__":
    threading.Thread(target=robot_thread, daemon=True).start()
    print("dashboard at http://localhost:%d/  (ctrl-c to stop)" % PORT)
    HTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
