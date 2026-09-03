# Maze Solver Robot - Pico W async web server + robot control loop.
#
# GOLDEN RULE: the dashboard only ever reads/writes the single shared `state`
# dict below. It never touches motors or sensors. The robot loop writes `state`,
# the web server serves it. That is the entire interface between the two halves.

import gc
import time
import network
import uasyncio as asyncio

import config
import maze

# --------------------------------------------------------------- shared state
state = {
    "pos": list(config.START),      # [row, col]
    "heading": "S",                 # "N" | "S" | "E" | "W"
    "discovered": ["%d,%d" % config.START],
    "explored": 1,
    "elapsed": 0.0,
    "message": "Standing by.",
    "algo": "floodfill",
    "running": False,
    "start": list(config.START),
    "goal": list(config.GOAL),
}

_run_started_ms = 0


def reset_state():
    global _run_started_ms
    state["pos"] = list(config.START)
    state["heading"] = "S"
    state["discovered"] = ["%d,%d" % config.START]
    state["explored"] = 1
    state["elapsed"] = 0.0
    state["message"] = "Standing by."
    state["running"] = False
    _run_started_ms = 0


def mark_discovered(r, c):
    k = "%d,%d" % (r, c)
    if k not in state["discovered"]:
        state["discovered"].append(k)
        state["explored"] = len(state["discovered"])


def _json(obj):
    """Tiny JSON encoder - ujson exists on the Pico, this keeps it dependency free."""
    import ujson
    return ujson.dumps(obj)


# ------------------------------------------------------------------ robot loop
STEP_PAUSE_MS = 700     # simulated time per cell; the real robot sets its own pace


async def robot_task():
    global _run_started_ms
    while True:
        if not state["running"]:
            await asyncio.sleep_ms(100)
            continue

        # vvvv REPLACE THIS BLOCK FOR THE REAL ROBOT vvvv
        # See docs/CONTEXT.md section 5. The pieces already exist in movement.py/sonar.py:
        #   - read 3 ultrasonics -> set state["message"] ("wall in front" etc.)
        #   - mark_discovered(r, c) for the current cell
        #   - drive one cell (~11925 counts) -> update state["pos"]
        #   - update state["heading"] on turns
        #   - let state["algo"] pick the next move
        #   - keep awaiting so the web server keeps serving
        r, c = state["pos"]
        mark_discovered(r, c)

        if (r, c) == tuple(config.GOAL):
            state["running"] = False
            state["message"] = "Goal reached."
            await asyncio.sleep_ms(100)
            continue

        nxt, heading = maze.next_move(state["algo"], (r, c), state["heading"])
        if nxt == (r, c):
            state["running"] = False
            state["message"] = "Stuck - no move available."
            await asyncio.sleep_ms(100)
            continue

        if maze.has_wall(r, c, state["heading"]):
            state["message"] = "wall in front"
        elif maze.has_wall(r, c, maze.LEFT_OF[state["heading"]]):
            state["message"] = "wall on left"
        elif maze.has_wall(r, c, maze.RIGHT_OF[state["heading"]]):
            state["message"] = "wall on right"
        else:
            state["message"] = "Driving to %d,%d" % nxt

        state["heading"] = heading
        await asyncio.sleep_ms(STEP_PAUSE_MS)
        state["pos"] = list(nxt)
        mark_discovered(*nxt)
        # ^^^^ END BLOCK ^^^^

        if _run_started_ms:
            state["elapsed"] = (time.ticks_ms() - _run_started_ms) / 1000.0
        await asyncio.sleep(0)


# ----------------------------------------------------------------- web server
def _read_file(name):
    try:
        with open(name, "rb") as f:
            return f.read()
    except OSError:
        return None


async def _send(writer, status, ctype, body, extra=""):
    if isinstance(body, str):
        body = body.encode()
    head = ("HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
            "Cache-Control: no-store\r\nConnection: close\r\n%s\r\n"
            % (status, ctype, len(body), extra))
    writer.write(head.encode())
    writer.write(body)
    await writer.drain()


async def handle(reader, writer):
    global _run_started_ms
    try:
        line = await reader.readline()
        if not line:
            return
        parts = line.decode().split()
        if len(parts) < 2:
            return
        method, target = parts[0], parts[1]
        while True:                              # drain headers
            h = await reader.readline()
            if not h or h == b"\r\n":
                break

        path, _, query = target.partition("?")

        if path == "/":
            page = _read_file("index.html")
            if page is None:
                await _send(writer, "500 Internal Server Error", "text/plain",
                            "index.html missing on the Pico filesystem")
            else:
                await _send(writer, "200 OK", "text/html; charset=utf-8", page)

        elif path == "/state":
            await _send(writer, "200 OK", "application/json", _json(state))

        elif path == "/select":
            algo = "floodfill"
            for kv in query.split("&"):
                k, _, v = kv.partition("=")
                if k == "algo":
                    algo = v
            if algo in maze.STEPPERS:
                state["algo"] = algo
                await _send(writer, "200 OK", "application/json", _json({"ok": True, "algo": algo}))
            else:
                await _send(writer, "400 Bad Request", "application/json",
                            _json({"ok": False, "error": "unknown algo"}))

        elif path == "/run":
            reset_state()
            _run_started_ms = time.ticks_ms()
            state["running"] = True
            state["message"] = "Running %s." % state["algo"]
            await _send(writer, "200 OK", "application/json", _json({"ok": True}))

        elif path == "/reset":
            reset_state()
            await _send(writer, "200 OK", "application/json", _json({"ok": True}))

        else:
            await _send(writer, "404 Not Found", "text/plain", "not found")
    except Exception as e:
        print("http error:", e)
    finally:
        try:
            await writer.wait_closed()
        except Exception:
            pass
        gc.collect()


# ----------------------------------------------------------------------- wifi
def connect_wifi(timeout_s=20):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("connecting to", config.WIFI_SSID)
        wlan.connect(config.WIFI_SSID, config.WIFI_PASS)
        t0 = time.time()
        while not wlan.isconnected():
            if time.time() - t0 > timeout_s:
                raise RuntimeError("wifi connect timed out")
            time.sleep(0.5)
    ip = wlan.ifconfig()[0]
    print("dashboard at http://%s/" % ip)
    return ip


async def main():
    connect_wifi()
    asyncio.create_task(robot_task())
    server = await asyncio.start_server(handle, "0.0.0.0", 80)
    print("server up")
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        asyncio.new_event_loop()
