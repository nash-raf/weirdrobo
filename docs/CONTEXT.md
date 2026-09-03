# Maze Solver Robot (Pico W) — Project Context

Handoff for anyone building or extending the WiFi dashboard that talks to the robot.

## 1. What it is

A maze-solving robot on a **Raspberry Pi Pico W**, in **MicroPython**. It navigates a
**4×4 maze** cell to cell, senses walls, and solves with one of three selectable
algorithms. A web dashboard served over WiFi shows the maze, live position, discovered
walls, telemetry, and lets the user pick the algorithm.

There is a parallel Raspberry Pi 4 version (separate effort). **This is the Pico W
version** — no camera, 264 KB RAM, MicroPython not full Python.

### Maze facts
- Grid **4×4** (16 cells).
- Cell interior **25 cm**, wall thickness **3 cm** → **pitch 28 cm** centre-to-centre.
  That is the distance driven per cell.
- **Start (S):** top-left, `(row 3, col 0)`. **Goal (E):** bottom-right, `(row 0, col 3)`.
- Convention: **row 0 = bottom, row 3 = top; col 0 = left, col 3 = right.**

## 2. Hardware

| Part | Detail |
|---|---|
| MCU | Raspberry Pi Pico W (MicroPython) |
| Motors | 2× N20 12V, 200:1, ~110 RPM rated, driven at 7.4V |
| Motor driver | TB6612FNG (dual H-bridge) |
| Encoders | Quadrature (2 channels each), both motors |
| Distance sensors | 3× HC-SR04+ (3.3V) — left, front, right |
| IMU (optional) | MPU-6050 (may or may not be fitted) |
| Wheels | 44 mm dia (138.23 mm circumference) |
| Power | 2S LiPo (7.4V) → buck 5V for logic; motors off 7.4V |

### Calibrated constants (MEASURED)
- **CPR = 5887** counts per wheel revolution (x4 quadrature; datasheet-implied 9600 was wrong).
- **0.02348 mm per count.**
- **~11925 counts = one 280 mm cell.** Verified on the floor (~28 cm travelled).

### GPIO map
| GPIO | Use | GPIO | Use |
|---|---|---|---|
| GP0 | Motor PWMA (left) | GP11 | Left sensor Trig |
| GP1 | AIN1 | GP12 | Left sensor Echo |
| GP2 | AIN2 | GP13 | Front sensor Trig |
| GP3 | BIN1 | GP14 | Front sensor Echo |
| GP4 | BIN2 | GP15 | Right sensor Trig |
| GP5 | Motor PWMB (right) | GP16 | Right sensor Echo |
| GP6 | STBY | GP17 | MPU INT (opt) |
| GP7 | Left encoder A | GP18 | I2C0 SDA (MPU) |
| GP8 | Left encoder B | GP19 | I2C0 SCL (MPU) |
| GP9 | Right encoder A | | |
| GP10 | Right encoder B | | |

## 3. Architecture

**Golden rule: the dashboard only ever reads/writes ONE shared `state` object.** It never
touches motors or sensors. The robot control loop updates `state`; the web server serves
it. This lets the UI and the robot logic be built independently.

```
   robot control loop  --writes-->  state (dict)  --served as JSON-->  browser
                                       ^                                   |
                                       +------ /select, /run, /reset ------+
```

Runs on **`uasyncio`** so the web server and robot loop share the single core
cooperatively — the robot loop must `await` regularly or the server stops responding.
The page **polls** a few times per second and redraws.

## 4. The data contract

`GET /state` returns:

```json
{
  "pos": [3, 0],
  "heading": "N",
  "discovered": ["3,0"],
  "explored": 1,
  "elapsed": 0.0,
  "message": "Standing by.",
  "algo": "floodfill",
  "running": false,
  "start": [3, 0],
  "goal": [0, 3]
}
```

- `pos` — `[row, col]`, drives the robot marker. Updated when a cell move completes.
- `heading` — `"N"|"S"|"E"|"W"`, which way it faces.
- `discovered` — `"row,col"` strings; the UI brightens a cell's walls once listed.
- `explored` — `len(discovered)`.
- `elapsed` — seconds since Run.
- `message` — free text; the UI styles it as an alert if it contains "wall".
- `algo` — set via `/select`; the robot reads it to pick its algorithm.
- `running` — true between Run and reaching the goal.

### Endpoints
| Method | Path | Effect |
|---|---|---|
| GET | `/` | serves `index.html` |
| GET | `/state` | the `state` JSON |
| POST | `/select?algo=floodfill\|lefthand\|astar` | sets `state["algo"]` |
| POST | `/run` | resets state, sets `running=true` |
| POST | `/reset` | back to standing by |

### Maze wall data
Known/pre-mapped layout (the robot is not discovering an unknown maze; walls are already
transcribed). Keyed `"row,col"` → wall sides. Symmetric, border walls included. Lives in
both `pico/maze.py` and `pico/index.html` — **keep the two copies in sync.**

```json
{"0,0":["E","S","W"],"0,1":["S","W"],"0,2":["E","S"],"0,3":["E","S","W"],
 "1,0":["W"],"1,1":["E","N"],"1,2":["W"],"1,3":["E","N"],
 "2,0":["N","W"],"2,1":["E","S"],"2,2":["E","W"],"2,3":["E","S","W"],
 "3,0":["N","S","W"],"3,1":["E","N"],"3,2":["N","W"],"3,3":["E","N"]}
```
> Confirm against the physical maze; it may be re-edited.

## 5. Where the robot logic plugs in

`pico/main.py` has one async `robot_task()` with a marked block. **That block is the only
thing that changes** between the simulated demo and the real robot:

```python
async def robot_task():
    # vvvv REPLACE THIS BLOCK FOR THE REAL ROBOT vvvv
    #   - read 3 ultrasonics -> set state["message"]
    #   - mark_discovered(r, c)
    #   - drive one cell (~11925 counts) -> update state["pos"]
    #   - update state["heading"] on turns
    #   - let state["algo"] pick the next move
    #   - keep awaiting so the web server keeps serving
    # ^^^^ END BLOCK ^^^^
```

The pieces it calls already exist: `movement.Movement.forward_cell()` / `.face()`,
`sonar.SonarArray.walls()`, `maze.next_move(algo, pos, heading)`.

## 6. Build status

**Working & verified on hardware:**
- Motors soldered, both drive forward (left flipped in software).
- Encoders soldered, both channels verified, calibrated (CPR 5887).
- Power: 2S LiPo → switch → buck 5V → Pico VSYS; motors on 7.4V via TB6612 VM.
- **Drive one cell straight** — encoder-balanced, ~28 cm accurate.

**Not yet done:**
- 90° turn — needs wheelbase measured, then `COUNTS_PER_90`. `movement.turn()` refuses
  to move until `config.COUNTS_PER_90` is set.
- HC-SR04+ integration into the loop + `WALL_THRESHOLD_MM` calibration.
- Swapping the simulated `robot_task()` block for the real movement loop.

**Done in software:** all three algorithms (flood fill, left-hand rule, A*), the state
contract, the server, the dashboard.

## 7. Practical notes

- **WiFi:** station mode; SSID/password at the top of `pico/config.py`. On boot it prints
  its IP; dashboard at `http://<ip>/`. AP mode is an alternative if no router — not used.
- **Keep it non-blocking:** anything server-side must be `async` and must not stall the
  robot loop. Note the sonar read is blocking (up to ~30 ms per sensor).
- **RAM is tight (264 KB):** keep the page light, no frameworks. No camera/video — that's
  a Pi 4 feature.
- **Polling ~5–7 Hz** is smooth enough; the robot hops cell to cell so the marker steps.
- **Don't invent new `state` fields silently** — agree the key with whoever owns the robot
  loop so both sides stay in sync.
- **No CORS concerns** while the page is served by the Pico itself (same origin).

## 8. Glossary
- **Cell pitch (28 cm):** distance driven to advance one cell.
- **CPR (5887):** encoder counts per wheel revolution (measured).
- **Counts per cell (~11925):** encoder target for one forward cell.
- **Quadrature x4:** counting both edges of both channels for max resolution + direction.
- **`state`:** the single shared dict that is the entire robot↔dashboard interface.
