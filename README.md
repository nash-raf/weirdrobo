# weirdrobo — maze-solving robot

A Raspberry Pi Pico W solves a 4×4 maze with the **left-hand rule** — read
three ultrasonics, always try to turn left, else straight, else right, else
turn around. It has no map and no idea where it is. It just reports what it
does, and a small web backend turns that stream into a live map, a run
history, and a database.

See **[ARCHITECTURE.md](ARCHITECTURE.md)** for exactly which file runs where
and how a run flows end to end. This file is the how-to-run and how-it-thinks.

## Repository layout

```
test/
  reactive_solver.py       flash this onto the Pico. self-contained MicroPython.
tools/
  telemetry_receiver.py    the backend hub - plain Python 3, stdlib only
  maze_db.py                SQLite: one row per run, one row per move
  dashboard.html             the web page the hub serves
  server.sh                  optional: deploy/manage the hub on a remote host
```

That's the whole system. Nothing else in the repo runs.

## Running it

**1. The backend**, anywhere with Python 3 (a laptop, a spare server, a Pi):

```
python3 tools/telemetry_receiver.py
# -> control hub on http://127.0.0.1:8787/
```

Override the port/host with `RX_PORT` / `RX_HOST` env vars. The database file
(`tools/weirdrobo.db`) is created next to `maze_db.py` on first run.

**2. The Pico.** Open `test/reactive_solver.py`, edit the top of the file:

```python
WIFI_SSID = "your-wifi-name"
WIFI_PASS = "your-wifi-password"

CONTROL_URLS = [
    "http://<HUB_HOST>:8787",   # wherever step 1 is running
]
```

`CONTROL_URLS` is tried in order at boot — list more than one and the robot
falls back automatically if the first is unreachable. Plain `http://` only:
MicroPython's `urequests` does not follow redirects, so never put an
`https://` URL in that list unless the backend itself is plain HTTP behind it.

Flash it (Thonny: open the file, pick the Pico, Run — or save as `main.py` to
run on power-up). It prints `link ok: <url>` once it finds the hub.

**3. Open the dashboard** at `http://<HUB_HOST>:8787/` and press **Start**.

## How it thinks

Every step, in strict priority:

```
if left is open      -> turn left,  then forward one cell
elif front is open   -> forward one cell
elif right is open   -> turn right, then forward one cell
else (dead end)       -> turn around, then forward one cell
```

This is only *guaranteed* to reach the goal in a **simply-connected** maze —
every wall attached to the outer boundary, no free-standing islands — with the
goal on the boundary. Check your own maze against that before trusting it.

The robot never tracks its own position — that's the whole appeal of a
reactive solver, and it's deliberately kept that way. The backend
(`telemetry_receiver.py`) integrates the stream of `turn`/`forward` events
into a cell + heading, and because it's the one holding coordinates, it's also
the one that notices when the robot has reached the goal and tells it to stop.

## Correcting drift, without position tracking

Two independent problems, two fixes, neither of which needs the robot to know
where it is:

- **Turn creep.** A 90° spin should pivot about the axle centre. If one wheel
  slips, the naive approach (wait for both wheels' *average* to reach target)
  leaves the robot under-rotated *and* pushed forward. Fix: each wheel stops on
  its own target, with a balance term keeping them together. The creep left
  over is still measurable — in a pure spin the wheels counter-rotate, so the
  **signed sum** of the two encoder counts is exactly the net forward slide —
  and it's driven straight back out.
- **Distance drift.** Dead-reckoned distance is fine for one cell and wrong by
  the tenth. Fix: after every forward move, if there's a wall ahead, square up
  to it at a fixed standoff (`FRONT_STOP_MM`). Every cell with a wall in front
  resets position to sonar accuracy, so error can't accumulate past it.

Both are bounded (`MAX_NUDGE_MM`) so one bad sonar reading can never drive the
robot into a wall, and both are behind `USE_CORRECTION` in
`reactive_solver.py` so you can A/B them in the real maze and compare the two
runs in `/runs`. Every correction is reported to the website too — the event
log and the database record lines like `turned right [crept +12.4mm, trimmed
-12.1mm]`, so you can show the robot correcting itself, with numbers.

## Calibration

These live at the top of `reactive_solver.py` — measure your own robot, don't
guess:

| Constant | What it is |
|---|---|
| `MM_PER_COUNT` | wheel travel per encoder count (wheel circumference ÷ counts-per-rev, both measured) |
| `CELL_CM` | maze cell pitch, centre to centre — everything else derives from this |
| `TURN_COUNTS` | encoder counts for a 90° spin, tuned on the floor |
| `WALL_THRESH_CM` | closer than this on a sonar = wall |
| `FRONT_STOP_MM` | correct standoff from a wall directly ahead |

## Runs are stored — SQLite

`tools/maze_db.py`, one file (`tools/weirdrobo.db`). SQLite because it's in
the Python standard library: no server, no user, no password, nothing to
install. Two tables — `runs` (one row per attempt) and `events` (one row per
sense/turn/forward, with the sonar readings behind each decision). `/runs`
replays any past attempt move by move; `python3 tools/maze_db.py` self-tests
the schema.

## Deploying the hub — `server.sh`

`tools/server.sh` is a small helper for running the hub on a remote host over
SSH (edit the `HOST`/`KEY` variables at the top for your own machine):

```
./tools/server.sh status     is it up?
./tools/server.sh log        recent hub log
./tools/server.sh deploy     push local changes and restart
./tools/server.sh db         run/event counts
```

It assumes the hub runs as a `systemd` service on the remote host so it
survives reboots and restarts on crash — see the comments in the script for
the unit file shape.

**Whatever you run it on, be deliberate about exposure.** The dashboard has no
login: anyone who can reach the port can press Start and drive the robot.
Fine for a demo you're standing next to; not fine left open indefinitely.
