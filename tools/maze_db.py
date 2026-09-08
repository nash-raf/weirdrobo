#!/usr/bin/env python3
"""SQLite store for maze runs.

Why SQLite and not MySQL: it is in the Python standard library, the whole
database is one file next to this script, and there is no server, no user and
no password to configure. Nothing to install on the demo machine.

Two tables, and the interesting one is `events`:

    runs    one row per attempt      - algo, when, how long, did it finish
    events  one row per THING THE ROBOT DID - every sense, turn and forward

So the website is not just showing a live position it forgets a second later:
every run is replayable afterwards from the database, which is what /runs uses.

Desktop-testable like the rest of tools/:  python3 tools/maze_db.py
"""
import os
import sqlite3
import time

DEFAULT_PATH = os.environ.get(
    "RX_DB", os.path.join(os.path.dirname(os.path.abspath(__file__)), "weirdrobo.db"))

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    algo          TEXT    NOT NULL,
    started_at    REAL    NOT NULL,
    ended_at      REAL,
    result        TEXT,               -- goal | stopped | stuck | NULL while running
    steps         INTEGER NOT NULL DEFAULT 0,
    cells_visited INTEGER NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS events (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id  INTEGER,                  -- NULL = sensed while standing by
    ts      REAL    NOT NULL,
    kind    TEXT    NOT NULL,         -- sense | forward | turn | goal | stuck | start | stop
    pos_r   INTEGER,
    pos_c   INTEGER,
    heading TEXT,
    status  TEXT,                     -- 'clear ahead' | 'wall ahead' | 'reached goal'
    sonar_l REAL,
    sonar_f REAL,
    sonar_r REAL,
    note    TEXT,
    FOREIGN KEY (run_id) REFERENCES runs(id)
);

CREATE INDEX IF NOT EXISTS idx_events_run ON events (run_id, id);
"""


def connect(path=DEFAULT_PATH):
    # check_same_thread=False: the hub is single-threaded today, but this way a
    # threaded server later cannot turn into a mystery crash.
    conn = sqlite3.connect(path, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.executescript(SCHEMA)
    conn.commit()
    return conn


# ------------------------------------------------------------------- writing
def start_run(conn, algo="lefthand", now=None):
    now = now if now is not None else time.time()
    cur = conn.execute(
        "INSERT INTO runs (algo, started_at) VALUES (?, ?)", (algo, now))
    conn.commit()
    return cur.lastrowid


def end_run(conn, run_id, result, steps=0, cells_visited=0, now=None):
    if run_id is None:
        return
    now = now if now is not None else time.time()
    conn.execute(
        "UPDATE runs SET ended_at=?, result=?, steps=?, cells_visited=? WHERE id=?",
        (now, result, steps, cells_visited, run_id))
    conn.commit()


def log_event(conn, run_id, kind, pos=None, heading=None, status=None,
              sonar=None, note=None, now=None):
    """One row per robot action. `pos` is (row, col); `sonar` is {L,F,R} in mm."""
    now = now if now is not None else time.time()
    sonar = sonar or {}
    conn.execute(
        "INSERT INTO events (run_id, ts, kind, pos_r, pos_c, heading, status,"
        " sonar_l, sonar_f, sonar_r, note) VALUES (?,?,?,?,?,?,?,?,?,?,?)",
        (run_id, now, kind,
         pos[0] if pos else None, pos[1] if pos else None,
         heading, status,
         sonar.get("L"), sonar.get("F"), sonar.get("R"), note))
    conn.commit()


# ------------------------------------------------------------------- reading
def recent_runs(conn, limit=20):
    rows = conn.execute(
        "SELECT r.*, (SELECT COUNT(*) FROM events e WHERE e.run_id=r.id) AS n_events"
        " FROM runs r ORDER BY r.id DESC LIMIT ?", (limit,)).fetchall()
    return [dict(r) for r in rows]


def run_events(conn, run_id, limit=500):
    rows = conn.execute(
        "SELECT * FROM events WHERE run_id=? ORDER BY id LIMIT ?",
        (run_id, limit)).fetchall()
    return [dict(r) for r in rows]


def totals(conn):
    """Headline numbers for the history page."""
    r = conn.execute(
        "SELECT COUNT(*) AS runs,"
        " SUM(result='goal') AS solved,"
        " MIN(CASE WHEN result='goal' THEN steps END) AS best_steps,"
        " MIN(CASE WHEN result='goal' THEN ended_at-started_at END) AS best_secs"
        " FROM runs").fetchone()
    out = dict(r)
    out["events"] = conn.execute("SELECT COUNT(*) AS n FROM events").fetchone()["n"]
    out["runs"] = out["runs"] or 0
    out["solved"] = out["solved"] or 0
    return out


# ---------------------------------------------------------------- self-test
if __name__ == "__main__":
    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "t.db")
    c = connect(path)

    rid = start_run(c, "lefthand", now=1000.0)
    log_event(c, rid, "start", pos=(3, 0), heading="E", now=1000.0)
    log_event(c, rid, "sense", pos=(3, 0), heading="E", status="clear ahead",
              sonar={"L": 90, "F": 400, "R": 300}, now=1001.0)
    log_event(c, rid, "forward", pos=(3, 1), heading="E", now=1002.0)
    log_event(c, rid, "sense", pos=(3, 1), heading="E", status="wall ahead",
              sonar={"L": 80, "F": 95, "R": 400}, now=1003.0)
    log_event(c, rid, "turn", pos=(3, 1), heading="S", note="right", now=1004.0)
    log_event(c, rid, "goal", pos=(0, 3), heading="S", status="reached goal",
              now=1010.0)
    end_run(c, rid, "goal", steps=2, cells_visited=2, now=1010.0)

    evs = run_events(c, rid)
    assert len(evs) == 6, evs
    assert evs[0]["kind"] == "start"
    assert evs[3]["status"] == "wall ahead" and evs[3]["sonar_f"] == 95
    assert evs[4]["note"] == "right" and evs[4]["heading"] == "S"

    runs = recent_runs(c)
    assert len(runs) == 1 and runs[0]["result"] == "goal", runs
    assert runs[0]["n_events"] == 6
    assert runs[0]["ended_at"] - runs[0]["started_at"] == 10.0

    # a second, abandoned run must not disturb the first
    rid2 = start_run(c, "lefthand", now=2000.0)
    log_event(c, rid2, "sense", pos=(3, 0), heading="E", status="clear ahead",
              now=2001.0)
    end_run(c, rid2, "stopped", steps=0, cells_visited=1, now=2005.0)
    assert [r["result"] for r in recent_runs(c)] == ["stopped", "goal"]

    # an event logged with no run (robot idle) is allowed and excluded from runs
    log_event(c, None, "sense", pos=(3, 0), status="clear ahead", now=3000.0)
    t = totals(c)
    assert t["runs"] == 2 and t["solved"] == 1, t
    assert t["events"] == 8, t
    assert t["best_steps"] == 2 and t["best_secs"] == 10.0, t

    print("maze_db self-test: all passed  (%s)" % path)
