# Where each file runs

Four files. Two machines. One database.

```
┌─────────────────────┐        HTTP/JSON        ┌──────────────────────────────┐
│   Raspberry Pi Pico W │ ───────────────────────▶│   backend hub (any server)   │
│                      │                          │                              │
│  reactive_solver.py  │◀─────────────────────────│  telemetry_receiver.py       │
│  (MicroPython)       │   queued MAZE / STOP      │  (Python 3, stdlib only)     │
└─────────────────────┘                           │       │                      │
                                                    │       ├─ maze_db.py         │
        ┌───────────────────────────────────┐      │       │  (SQLite)           │
        │           a browser                │◀────┤       │                     │
        │  shows dashboard.html, polls       │      │       └─ dashboard.html    │
        │  /dash-state, presses Start/Stop   │      │          (served as-is)    │
        └───────────────────────────────────┘      └──────────────────────────────┘
```

| File | Runs on | What it is |
|---|---|---|
| **`test/reactive_solver.py`** | **the Pico** (flash this, and only this) | Left-hand-rule maze solver. Reads its own 3 ultrasonics, decides every turn itself, drives the motors, and reports each action to the hub over HTTP. Standalone — imports only `machine`, `time`, `network`, `urequests`. |
| **`tools/telemetry_receiver.py`** | **the backend server** | Plain Python 3, no dependencies beyond the standard library. Serves the web dashboard, tracks the robot's position from the actions it reports (the robot itself has no idea where it is), decides when it's reached the goal, and logs every run to SQLite. |
| **`tools/maze_db.py`** | imported by `telemetry_receiver.py` | The database layer. One SQLite file, two tables: `runs` (one row per attempt) and `events` (one row per move/turn/sense). No server, no setup — it's the Python standard library's `sqlite3`. |
| **`tools/dashboard.html`** | served by `telemetry_receiver.py` at `/maze_runner` | The web page a browser sees: a live 4×4 map, the "clear ahead / wall ahead" banner, the Start/Stop buttons, and a run history link. Plain HTML/CSS/JS, no build step. |
| `tools/server.sh` | run from your machine, manages the server | Convenience script — deploy/restart/status/log for `telemetry_receiver.py` wherever it's hosted. Not part of the running system, just ops tooling. |

## How a run actually happens

1. A browser opens `/maze_runner` (served from `dashboard.html`) and presses **Start**.
2. That's `POST /run-maze` — the hub opens a row in SQLite and queues a `MAZE`
   job.
3. The Pico polls `GET /cmd-state` every couple of seconds. It sees the job,
   waits 2 s, and starts solving: read the 3 sonars → left-hand rule decides
   turn/straight → drive → repeat.
4. Every action gets `POST /event`'d to the hub — `sense`, `turn`, `forward`.
   The robot moves, *then* the dot on the browser's map moves.
5. The robot never tracks its own position. `telemetry_receiver.py` turns the
   stream of turns/forwards into a cell + heading (`_integrate()`), because a
   pure reactive solver shouldn't need to.
6. When the hub's own map reaches the goal cell, **the hub** decides the run
   is over, closes it in SQLite, and queues a `STOP` for the robot to pick up
   on its next poll.
7. `/runs` reads the database back — every past attempt, and the full
   move-by-move log of any one you click into.

## Endpoints

| Method | Path | Called by |
|---|---|---|
| GET | `/` | anyone — redirects to `/maze_runner` |
| GET | `/maze_runner` | browser — the dashboard page |
| GET | `/dash-state` | browser — polls for the live map JSON |
| POST | `/run-maze` | browser — Start button |
| POST | `/stop-maze` | browser — Stop button |
| GET | `/runs`, `/runs?id=N` | browser — run history |
| GET | `/runs-json` | browser — same, as JSON |
| GET | `/healthz` | ops — liveness check |
| GET | `/cmd-state` | **Pico** — polls for a queued job |
| POST | `/event` | **Pico** — reports one action |
| POST | `/status` | **Pico** — idle sonar readings between runs |
| POST | `/cmd-done` | **Pico** — acks a finished MAZE/STOP |

That's the whole system — nothing else in the repo runs.
