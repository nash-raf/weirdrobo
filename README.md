# weirdrobo — Maze Solver Robot (Raspberry Pi Pico W)

A 4×4 maze-solving robot in MicroPython, with a WiFi dashboard showing live
position, discovered walls, and telemetry.

```
pico/
  main.py       async web server + robot control loop (owns the shared `state`)
  index.html    the dashboard (falls back to a built-in simulator with no Pico)
  runner.py     run lifecycle, algorithm latching, telemetry  (desktop-testable)
  solver.py     one-cell-at-a-time solving from any start cell (desktop-testable)
  maze.py       known maze map, the three algorithms, turn geometry
  config.py     GPIO map, measured calibration, drive limits, PID gains
  hardware.py   HC-SR04+ / quadrature encoders / TB6612FNG  (MicroPython only)
  motion.py     PID cell drive, 90° turns, command timeouts  (MicroPython only)
tools/
  test_solver.py, test_runner.py, serve_local.py
docs/
  CONTEXT.md    full project handoff: hardware, contract, build status
```

`hardware.py` and `motion.py` are adapted from `firmware/` on
[samiulislam07/autonomous-maze-solving-robot @ Samiul](https://github.com/samiulislam07/autonomous-maze-solving-robot/tree/Samiul),
re-pointed at this project's measured calibration. The motion primitives were
made `async` — on this build the web server shares the single core with the
robot loop, so a blocking move would freeze the dashboard for the whole 28 cm.

## Quick look at the dashboard
Open `pico/index.html` in a browser. With no Pico reachable it auto-detects and
runs a simulated robot walking the solution path.

## On the robot
1. Set `WIFI_SSID` / `WIFI_PASS` in `pico/config.py`.
2. Copy everything in `pico/` to the Pico W filesystem.
3. Reset. It prints `dashboard at http://<ip>/`.

## The contract
The dashboard only ever reads/writes one shared `state` dict. It never touches
motors or sensors. See `docs/CONTEXT.md` §4.

| Method | Path | Effect |
|---|---|---|
| GET | `/` | dashboard page |
| GET | `/state` | the `state` JSON |
| POST | `/select?algo=floodfill\|lefthand\|astar` | choose algorithm |
| POST | `/run` | reset + start solving |
| POST | `/reset` | back to standing by |

## Maze
4×4, cell interior 25 cm, wall 3 cm, **pitch 28 cm/cell**.
Start `(3,0)` top-left, goal `(0,3)` bottom-right.
Coordinates are `(row, col)` with **row 0 = bottom, row 3 = top**.

## Calibration (measured, not datasheet)
- CPR **5887** counts/wheel rev (x4 quadrature)
- **0.02348 mm** per count
- **11925 counts** = one 280 mm cell — verified on the floor

## Testing without hardware

```
python3 tools/test_solver.py    # 192 runs: 16 start cells x 4 headings x 3 algos
python3 tools/test_runner.py    # run lifecycle: latching, mid-run lock, reset
python3 tools/serve_local.py    # the real dashboard + real logic on localhost:8080
```

`serve_local.py` reuses `runner.py` and `index.html` unchanged, so it exercises the
actual contract rather than a mock of it. Only the motors are missing.

## The algorithm is latched per run

A run finishes with the algorithm it started with. `/run` latches `state["algo"]` into
`state["active_algo"]`; `/select` returns **409** while `running`, and the dashboard
dims the buttons. `/reset` is the way to abort and switch.

## Status
Working: motors, encoders (calibrated), power chain, straight one-cell drive, all three
algorithms (tested from every cell), run lifecycle + lock, dashboard, server plumbing.
Remaining: 90° turn calibration (needs wheelbase), sonar integration, swapping the
simulated block in `robot_task()` for the real movement loop.
