# weirdrobo — Maze Solver Robot (Raspberry Pi Pico W)

A 4×4 maze-solving robot in MicroPython, with a WiFi dashboard showing live
position, discovered walls, and telemetry.

```
pico/
  main.py       async web server + robot control loop (owns the shared `state`)
  index.html    the dashboard (falls back to a built-in simulator with no Pico)
  maze.py       known maze map + flood fill / left-hand rule / A*
  config.py     GPIO map + measured calibration constants
  motors.py     TB6612FNG driver
  encoders.py   x4 quadrature decoding
  sonar.py      3× HC-SR04+ (left / front / right)
  movement.py   one-cell drive, 90° turn
docs/
  CONTEXT.md    full project handoff: hardware, contract, build status
```

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

## Status
Working: motors, encoders (calibrated), power chain, straight one-cell drive,
all three algorithms, dashboard, server + state plumbing.
Remaining: 90° turn calibration (needs wheelbase), sonar integration, swapping
the simulated `robot_task()` block for the real movement loop.
