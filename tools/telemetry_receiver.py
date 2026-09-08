#!/usr/bin/env python3
"""weirdrobo maze hub — the backend behind reactive_solver.py and dashboard.html.

Two directions of traffic:
  * The Pico posts what it just did (POST /event) as it wall-follows the maze.
    This server does the dead reckoning — the robot only knows "left is open,
    so I turn left"; turning that stream into a position on a map happens
    here, not on the Pico. See _integrate() / _apply_event().
  * Idle, between runs, the Pico still posts its sonar readings (POST /status)
    so the dashboard's "clear ahead / wall ahead" banner stays honest.

The browser opens /maze_runner, presses Start, and watches the map update
live. Every run and every move is written to SQLite (maze_db.py) and readable
afterwards at /runs.

Endpoints, and who calls them:
    GET  /                bare URL -> redirect to /maze_runner
    GET  /maze_runner      serves dashboard.html                 (browser)
    GET  /dash-state       live map state as JSON                (browser, polls)
    POST /run-maze         Start: open a DB run, queue MAZE       (browser)
    POST /stop-maze        Stop: close the run, queue STOP        (browser)
    GET  /runs, /runs?id=N run history from SQLite                (browser)
    GET  /runs-json        same, as JSON                          (browser)
    GET  /healthz          liveness check                         (ops)
    GET  /cmd-state        "is there a job for me?"                (Pico, polls)
    POST /event            "I sensed / turned / moved forward"     (Pico)
    POST /status           idle sonar readings                     (Pico)
    POST /cmd-done         Pico acks a finished MAZE/STOP           (Pico)
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import maze_db

HOST = os.environ.get("RX_HOST", "127.0.0.1")
PORT = int(os.environ.get("RX_PORT", "8787"))

HERE = os.path.dirname(os.path.abspath(__file__))
DB = maze_db.connect()

PICO_SEEN = None      # last time the Pico made any request - freshness for /dash-state

# One command is queued at a time; the Pico takes it and acks it via /cmd-done.
QUEUED = None         # {"cmd": "MAZE"|"STOP", "seq": N, "at": ts, "reason": ...}
_seq = 0

GRID = 4
MAZE_START = [3, 0]
MAZE_GOAL = [0, 3]
START_HEADING = "E"   # (3,0) is walled N, S and W - east is the only way out
WALL_MM = 150         # closer than this on a sonar = wall
ALGO = "lefthand"     # the only algorithm this build runs


def _fresh_dash():
    return {"run_id": None, "running": False,
            "pos": list(MAZE_START), "heading": START_HEADING,
            "visited": ["%d,%d" % tuple(MAZE_START)],
            "trail": [list(MAZE_START)],
            "found_walls": [],
            "status": "standing by", "status_kind": "",
            "status_sub": "waiting for the Pico",
            "sonar_mm": {}, "steps": 0, "actions": 0,
            "started_at": None, "ts": None}


DASH = _fresh_dash()
EVENTS = []           # live log for the page: {"n","ts","kind","text"}
MAX_EVENTS = 60
_event_n = 0


# Dead reckoning for a robot that does not do its own.
# reactive_solver.py is a pure wall-follower: it knows "left is open so I
# turned left" and nothing else. Turning that stream into a position belongs
# HERE, next to the map that draws it - the robot stays dumb on purpose.
_TURN_LEFT = {"N": "W", "W": "S", "S": "E", "E": "N"}
_TURN_RIGHT = {"N": "E", "E": "S", "S": "W", "W": "N"}
_DELTA = {"N": (1, 0), "S": (-1, 0), "E": (0, 1), "W": (0, -1)}
_REL_SIDE = {"F": lambda h: h, "L": lambda h: _TURN_LEFT[h], "R": lambda h: _TURN_RIGHT[h]}


def _integrate(kind, note):
    """Advance the hub's idea of where the robot is, from what it just did."""
    word = (note or "").lower()
    if kind == "turn":
        if "around" in word:
            DASH["heading"] = _TURN_LEFT[_TURN_LEFT[DASH["heading"]]]
        elif "right" in word:
            DASH["heading"] = _TURN_RIGHT[DASH["heading"]]
        elif "left" in word:
            DASH["heading"] = _TURN_LEFT[DASH["heading"]]
    elif kind == "forward":
        dr, dc = _DELTA[DASH["heading"]]
        DASH["pos"] = [min(GRID - 1, max(0, DASH["pos"][0] + dr)),
                       min(GRID - 1, max(0, DASH["pos"][1] + dc))]


# The server is threaded, so two /event posts can land at the same instant.
# Every function that mutates the shared run state runs under this lock -
# without it the dead reckoning interleaves and the map ends up somewhere the
# robot never went. RLock because _apply_event calls _end_maze_run.
STATE_LOCK = threading.RLock()


def _locked(fn):
    """Serialise a state-mutating handler."""
    def wrapper(*args, **kwargs):
        with STATE_LOCK:
            return fn(*args, **kwargs)
    return wrapper


def _cell(pos):
    return "(%d,%d)" % (pos[0], pos[1])


def _mm(v):
    if v is None:
        return "-"
    return "open" if v >= 1500 else "%dmm" % round(v)


def _push_event(kind, text, ts):
    global _event_n
    _event_n += 1
    EVENTS.append({"n": _event_n, "ts": ts, "kind": kind, "text": text})
    if len(EVENTS) > MAX_EVENTS:
        del EVENTS[:-MAX_EVENTS]


def _ahead_from_sonar(sonar):
    """The one line the demo is about: is the way forward clear?"""
    front = sonar.get("F") if isinstance(sonar, dict) else None
    if front is None:
        return None, None, None
    if front < WALL_MM:
        return "wall ahead", "wall", "front sonar reads %s" % _mm(front)
    return "clear ahead", "clear", "front sonar reads %s" % _mm(front)


@_locked
def _start_maze_run(stamp):
    """Browser pressed Start. Open a DB run, reset the map, queue it for the Pico."""
    global DASH, EVENTS, QUEUED, _seq
    if DASH["running"]:
        return {"ok": False, "error": "a run is already going"}
    DASH = _fresh_dash()
    del EVENTS[:]
    DASH["run_id"] = maze_db.start_run(DB, ALGO, now=stamp)
    DASH["running"] = True
    DASH["started_at"] = stamp
    DASH["ts"] = stamp
    DASH["status"] = "starting"
    DASH["status_sub"] = "waiting for the Pico to pick the run up"
    maze_db.log_event(DB, DASH["run_id"], "start", pos=MAZE_START,
                      heading=START_HEADING, note=ALGO, now=stamp)
    _push_event("start", "run %d queued - left-hand rule from %s"
                % (DASH["run_id"], _cell(MAZE_START)), stamp)
    _seq += 1
    QUEUED = {"cmd": "MAZE", "seq": _seq, "at": stamp}
    print("[%s] browser started maze run %d (seq %d)"
          % (_now(), DASH["run_id"], _seq), flush=True)
    return {"ok": True, "run_id": DASH["run_id"], "seq": _seq}


@_locked
def _end_maze_run(result, stamp, note=None):
    global QUEUED, _seq
    if not DASH["running"]:
        return {"ok": False, "error": "nothing is running"}
    maze_db.end_run(DB, DASH["run_id"], result, steps=DASH["steps"],
                    cells_visited=len(DASH["visited"]), now=stamp)
    maze_db.log_event(DB, DASH["run_id"], "stop", pos=DASH["pos"],
                      heading=DASH["heading"], note=note or result, now=stamp)
    DASH["running"] = False
    DASH["ts"] = stamp
    if result != "goal":
        DASH["status"] = "stopped"
        DASH["status_kind"] = ""
        DASH["status_sub"] = note or "run ended after %d moves" % DASH["steps"]
    # The robot may still be driving. Leave a STOP for it to find on its next
    # between-moves poll, and say WHY, so it can report the run honestly
    # instead of calling a finished maze "stopped".
    _seq += 1
    QUEUED = {"cmd": "STOP", "seq": _seq, "at": stamp, "reason": result}
    _push_event("stop", "run %s ended: %s (%d cells moved, %d turns, %d cells seen)"
                % (DASH["run_id"], result, DASH["steps"],
                   DASH["actions"] - DASH["steps"], len(DASH["visited"])),
                stamp)
    print("[%s] maze run %s ended: %s" % (_now(), DASH["run_id"], result), flush=True)
    return {"ok": True, "result": result}


@_locked
def _apply_event(body, stamp):
    """One thing the Pico did. Updates the live map and writes a DB row."""
    kind = (body.get("kind") or "").lower()
    arrived = False
    pos = body.get("pos")
    heading = body.get("heading")
    sonar = body.get("sonar") or {}
    note = body.get("note")

    if isinstance(pos, (list, tuple)) and len(pos) == 2:
        DASH["pos"] = [int(pos[0]), int(pos[1])]      # robot tracks its own
    if heading:
        DASH["heading"] = heading
    if pos is None and heading is None:
        _integrate(kind, note)                        # robot does not - we do
    if sonar:
        DASH["sonar_mm"] = sonar
    DASH["ts"] = stamp

    key = "%d,%d" % (DASH["pos"][0], DASH["pos"][1])

    if kind == "sense":
        # `walls` is absolute sides; `rel_walls` is L/F/R from a robot that has
        # no idea which way is north - we resolve those against our heading.
        rel = body.get("rel_walls") or []
        walls = list(body.get("walls") or [])
        for r in rel:
            fn = _REL_SIDE.get(r)
            if fn:
                walls.append(fn(DASH["heading"]))
        for side in walls:
            wall_key = "%s,%s" % (key, side)
            if wall_key not in DASH["found_walls"]:
                DASH["found_walls"].append(wall_key)
        status, kindv, sub = _ahead_from_sonar(sonar)
        if status:
            DASH["status"], DASH["status_kind"], DASH["status_sub"] = status, kindv, sub
        _push_event("sense", "%s at %s  ·  L %s  F %s  R %s"
                    % (status or "sensed", _cell(DASH["pos"]),
                       _mm(sonar.get("L")), _mm(sonar.get("F")), _mm(sonar.get("R"))),
                    stamp)

    elif kind == "forward":
        if key not in DASH["visited"]:
            DASH["visited"].append(key)
        if not DASH["trail"] or DASH["trail"][-1] != DASH["pos"]:
            DASH["trail"].append(list(DASH["pos"]))
        DASH["steps"] += 1
        DASH["actions"] += 1
        DASH["status"], DASH["status_kind"] = "moving forward", "clear"
        DASH["status_sub"] = "now in %s facing %s" % (_cell(DASH["pos"]), DASH["heading"])
        _push_event("forward", "forward one cell to %s, facing %s"
                    % (_cell(DASH["pos"]), DASH["heading"]), stamp)
        # A robot that does not track position cannot know it has arrived.
        # The map does, so the hub calls it - after this move is on record.
        arrived = DASH["pos"] == MAZE_GOAL and DASH["running"]

    elif kind == "turn":
        DASH["actions"] += 1
        DASH["status"], DASH["status_kind"] = "turning %s" % (note or ""), "wall"
        DASH["status_sub"] = "wall ahead - now facing %s" % DASH["heading"]
        _push_event("turn", "turned %s, now facing %s"
                    % (note or "?", DASH["heading"]), stamp)

    elif kind == "goal":
        DASH["status"], DASH["status_kind"] = "reached goal", "goal"
        DASH["status_sub"] = "%s in %d moves" % (_cell(DASH["pos"]), DASH["steps"])
        if key not in DASH["visited"]:
            DASH["visited"].append(key)
        _push_event("goal", "REACHED THE GOAL at %s in %d moves"
                    % (_cell(DASH["pos"]), DASH["steps"]), stamp)

    elif kind == "stuck":
        DASH["status"], DASH["status_kind"] = "stuck", "wall"
        DASH["status_sub"] = note or "no way out of %s" % _cell(DASH["pos"])
        _push_event("stuck", note or "stuck at %s" % _cell(DASH["pos"]), stamp)

    else:
        _push_event(kind or "event", note or json.dumps(body)[:120], stamp)

    maze_db.log_event(DB, DASH["run_id"], kind or "event", pos=DASH["pos"],
                      heading=DASH["heading"], status=DASH["status"],
                      sonar=sonar, note=note, now=stamp)

    if arrived:
        DASH["status"], DASH["status_kind"] = "reached goal", "goal"
        DASH["status_sub"] = "%s in %d moves" % (_cell(DASH["pos"]), DASH["steps"])
        _push_event("goal", "REACHED THE GOAL at %s in %d moves"
                    % (_cell(DASH["pos"]), DASH["steps"]), stamp)
        maze_db.log_event(DB, DASH["run_id"], "goal", pos=DASH["pos"],
                          heading=DASH["heading"], status="reached goal",
                          now=stamp)
        _end_maze_run("goal", stamp, note="reached %s" % _cell(DASH["pos"]))

    # the robot itself declares the run over
    if kind == "goal":
        _end_maze_run("goal", stamp, note="reached %s" % _cell(DASH["pos"]))
    elif kind == "stuck":
        _end_maze_run("stuck", stamp, note=note)

    return {"ok": True, "running": DASH["running"], "run_id": DASH["run_id"]}


def _now():
    return time.strftime("%H:%M:%S")


RUNS_PAGE_HEAD = """<!doctype html><html><head><meta charset="utf-8">
<title>Maze Robot - run history</title>
<style>
:root{--paper:#f7f5ef;--card:#fff;--line:#e2ddd0;--ink:#1f2a33;--dim:#8a8577;
      --robot:#0e9e8e;--goal:#e5674f;--gold:#b8860b}
*{box-sizing:border-box}
body{margin:0;background:var(--paper);color:var(--ink);padding:24px;
     font-family:"IBM Plex Mono",ui-monospace,Menlo,Consolas,monospace;
     display:flex;justify-content:center}
.shell{width:100%;max-width:1000px}
header{display:flex;align-items:baseline;justify-content:space-between;
  border-bottom:1.5px solid var(--ink);padding-bottom:12px;margin-bottom:20px;gap:10px;flex-wrap:wrap}
h1{font-family:"Space Grotesk",system-ui,sans-serif;font-size:21px;margin:0;letter-spacing:.5px}
a{color:var(--robot);text-decoration:none}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:18px;margin-bottom:18px}
h2{font-size:11px;letter-spacing:2px;text-transform:uppercase;color:var(--dim);margin:0 0 14px;font-weight:600;
   font-family:"Space Grotesk",system-ui,sans-serif}
.tot{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:12px}
.tot div{background:var(--paper);border:1px solid var(--line);border-radius:10px;padding:12px}
.tot .k{font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim)}
.tot .v{font-size:25px;margin-top:4px;font-variant-numeric:tabular-nums}
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;font-size:10px;letter-spacing:1.5px;text-transform:uppercase;color:var(--dim);
   border-bottom:1px solid var(--line);padding:8px 6px;font-weight:600}
td{padding:8px 6px;border-bottom:1px dashed var(--line);font-variant-numeric:tabular-nums}
tr:hover td{background:#fbfaf6}
.pill{font-size:10px;letter-spacing:1px;text-transform:uppercase;padding:3px 8px;border-radius:99px;border:1px solid}
.pill.goal{color:#0b7d70;border-color:var(--robot);background:#eafaf7}
.pill.stopped{color:var(--dim);border-color:var(--line)}
.pill.stuck,.pill.running{color:#a23b2a;border-color:var(--goal);background:#fdf1ee}
.empty{color:var(--dim);font-size:13px;padding:10px 0}
</style></head><body><div class="shell">
<header><h1>MAZE ROBOT &middot; RUN HISTORY</h1>
<div><a href="/maze_runner">&larr; live maze</a></div></header>
"""

RUNS_PAGE_FOOT = """<p style="color:#8a8577;font-size:11px;text-align:center">
Stored in SQLite (tools/weirdrobo.db) - one row per run, one row per move.
Raw JSON: <a href="/runs-json">/runs-json</a></p>
</div></body></html>"""


def _fmt_ts(t):
    if not t:
        return "-"
    return time.strftime("%d %b %H:%M:%S", time.localtime(t))


def _runs_html(run_id=None):
    out = [RUNS_PAGE_HEAD]
    t = maze_db.totals(DB)
    best_steps = t["best_steps"] if t["best_steps"] is not None else "-"
    best_secs = ("%.1fs" % t["best_secs"]) if t["best_secs"] is not None else "-"
    out.append('<div class="card"><h2>Totals</h2><div class="tot">'
               '<div><div class="k">Runs</div><div class="v">%s</div></div>'
               '<div><div class="k">Solved</div><div class="v">%s</div></div>'
               '<div><div class="k">Fewest moves</div><div class="v">%s</div></div>'
               '<div><div class="k">Fastest</div><div class="v">%s</div></div>'
               '<div><div class="k">Events logged</div><div class="v">%s</div></div>'
               '</div></div>' % (t["runs"], t["solved"], best_steps, best_secs, t["events"]))

    runs = maze_db.recent_runs(DB, limit=25)
    out.append('<div class="card"><h2>Runs</h2>')
    if not runs:
        out.append('<div class="empty">No runs yet. Press Start on the '
                   '<a href="/maze_runner">live maze</a>.</div>')
    else:
        out.append('<table><tr><th>#</th><th>Started</th><th>Algorithm</th>'
                   '<th>Result</th><th>Moves</th><th>Cells</th><th>Time</th><th>Log</th></tr>')
        for r in runs:
            result = r["result"] or "running"
            secs = ("%.1fs" % (r["ended_at"] - r["started_at"])) if r["ended_at"] else "-"
            out.append('<tr><td>%d</td><td>%s</td><td>%s</td>'
                       '<td><span class="pill %s">%s</span></td>'
                       '<td>%d</td><td>%d</td><td>%s</td>'
                       '<td><a href="/runs?id=%d">%d events</a></td></tr>'
                       % (r["id"], _fmt_ts(r["started_at"]), r["algo"],
                          result, result, r["steps"], r["cells_visited"],
                          secs, r["id"], r["n_events"]))
        out.append('</table>')
    out.append('</div>')

    if run_id:
        evs = maze_db.run_events(DB, run_id)
        out.append('<div class="card"><h2>Run %d &middot; every move</h2>' % run_id)
        if not evs:
            out.append('<div class="empty">No events for that run.</div>')
        else:
            out.append('<table><tr><th>Time</th><th>What</th><th>Cell</th><th>Facing</th>'
                       '<th>Status</th><th>Sonar L/F/R (mm)</th><th>Note</th></tr>')
            for e in evs:
                cell = ("(%s,%s)" % (e["pos_r"], e["pos_c"])) if e["pos_r"] is not None else "-"
                sonar = " / ".join("-" if e[k] is None else str(round(e[k]))
                                   for k in ("sonar_l", "sonar_f", "sonar_r"))
                out.append('<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td>'
                           '<td>%s</td><td>%s</td><td>%s</td></tr>'
                           % (_fmt_ts(e["ts"]), e["kind"], cell, e["heading"] or "-",
                              e["status"] or "-", sonar, e["note"] or ""))
            out.append('</table>')
        out.append('</div>')

    out.append(RUNS_PAGE_FOOT)
    return "".join(out)


def _serve_file(name):
    with open(os.path.join(HERE, name)) as f:
        return f.read()


def _query(u):
    qs = {}
    for kv in u.query.split("&"):
        if not kv:
            continue
        k, _, v = kv.partition("=")
        qs[k] = v
    return qs


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
        global PICO_SEEN
        u = urlparse(self.path)
        if u.path == "/":
            # the bare domain is what gets shown to people: send it to the demo
            self.send_response(303)
            self.send_header("Location", "/maze_runner")
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif u.path == "/healthz":
            self._json({"ok": True})
        elif u.path in ("/maze_runner", "/dashboard"):
            try:
                self._send(200, "text/html; charset=utf-8", _serve_file("dashboard.html"))
            except OSError:
                self._send(500, "text/plain", "tools/dashboard.html is missing")
        elif u.path == "/dash-state":
            with STATE_LOCK:
                now = time.time()
                fresh = PICO_SEEN is not None and (now - PICO_SEEN) < 6
                d = dict(DASH)
                d["elapsed"] = (min(DASH["ts"] or now, now) - DASH["started_at"]) \
                    if DASH["started_at"] else 0.0
                d["link"] = "pico · live" if fresh else (
                    "waiting for pico" if PICO_SEEN else "no pico yet")
                d["events"] = EVENTS[-40:]
                d["algo"] = ALGO
            self._json(d)
        elif u.path == "/runs":
            qs = _query(u)
            rid = qs.get("id")
            try:
                rid = int(rid) if rid else None
            except ValueError:
                rid = None
            self._send(200, "text/html; charset=utf-8", _runs_html(rid))
        elif u.path == "/runs-json":
            self._json({"totals": maze_db.totals(DB),
                        "runs": maze_db.recent_runs(DB, limit=50)})
        elif u.path == "/cmd-state":
            # the Pico polls this every ~2s; the poll itself is the heartbeat.
            PICO_SEEN = time.time()
            self._json({"cmd": QUEUED})
        else:
            self._send(404, "text/plain", "not found")

    def do_POST(self):
        global PICO_SEEN, QUEUED
        u = urlparse(self.path)
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        try:
            body = json.loads(raw.decode()) if raw else {}
        except ValueError:
            body = {"_raw": raw.decode(errors="replace")}
        stamp = time.time()

        if u.path == "/cmd-done":
            # Pico finished (or failed) the queued MAZE/STOP.
            seq = body.get("seq")
            if QUEUED and QUEUED["seq"] == seq:
                QUEUED = None
            print("[%s] pico ack %s seq=%s ok=%s %s" % (
                _now(), body.get("cmd"), seq, body.get("ok"),
                json.dumps(body.get("report") or body.get("error") or "")), flush=True)
            self._json({"ok": True})
            return

        if u.path == "/run-maze":
            self._json(_start_maze_run(stamp))
            return

        if u.path == "/stop-maze":
            self._json(_end_maze_run("stopped", stamp, note="stopped from the website"))
            return

        if u.path == "/event":
            # one thing the Pico just did, during a maze run
            PICO_SEEN = stamp
            self._json(_apply_event(body if isinstance(body, dict) else {}, stamp))
            return

        if u.path == "/status":
            # Unsolicited push from the Pico between runs. Counts as a
            # heartbeat, and keeps the "clear ahead / wall ahead" banner
            # honest by carrying the idle sonar readings.
            with STATE_LOCK:
                PICO_SEEN = stamp
                sonar = body.get("sonar_mm") if isinstance(body, dict) else None
                if isinstance(sonar, dict) and not DASH["running"]:
                    DASH["sonar_mm"] = sonar
                    text, kindv, sub = _ahead_from_sonar(sonar)
                    # "reached goal" is the result of a run and stays on the
                    # banner until the next one starts - idle pings must not
                    # erase it.
                    if text and DASH["status_kind"] != "goal":
                        DASH["status"], DASH["status_kind"] = text, kindv
                        DASH["status_sub"] = sub + " (standing by)"
                        DASH["ts"] = stamp
            self._json({"ok": True})
            return

        self._send(404, "text/plain", "not found")


if __name__ == "__main__":
    print("control hub on http://%s:%d/  (ctrl-c to stop)" % (HOST, PORT))
    print("  maze runner     http://%s:%d/maze_runner" % (HOST, PORT))
    print("  run history     http://%s:%d/runs" % (HOST, PORT))
    print("  database        %s" % maze_db.DEFAULT_PATH)
    try:
        # Threaded on purpose. The dashboard polls /dash-state every 400 ms and
        # the robot polls /cmd-state every 2 s; with a single-threaded server
        # those queue behind each other and the Pico times out. sqlite is
        # opened with check_same_thread=False for exactly this.
        ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
