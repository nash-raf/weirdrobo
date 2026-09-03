# Maze Solver Robot - Pico W async web server + robot control loop.
#
# GOLDEN RULE: the dashboard only ever reads/writes the single shared `state`
# dict. It never touches motors or sensors. The robot loop writes `state`, the
# web server serves it. That is the entire interface between the two halves.
#
# The run logic lives in runner.py / solver.py, which have no MicroPython
# dependencies and are tested on a desktop by tools/test_runner.py. This file
# is only the async plumbing.

import gc
import network
import uasyncio as asyncio
import ujson

import config
from runner import Runner, ALGOS

RUN = Runner()

STEP_PAUSE_MS = 700         # simulated pace per cell; the real robot sets its own


# ------------------------------------------------------------------ robot loop
async def robot_task():
    while True:
        if not RUN.state["running"]:
            await asyncio.sleep_ms(100)
            continue

        # vvvv REPLACE THIS BLOCK FOR THE REAL ROBOT vvvv
        # RUN.advance() decides WHERE to go next and updates `state`. The only
        # thing missing here is physically going there. For the real robot:
        #
        #   target = RUN.solver.pos          # before advance: where we are
        #   RUN.advance()                    # -> RUN.state["heading"], ["pos"]
        #   await mv.face(old_heading, RUN.state["heading"])
        #   await mv.forward_cell()          # ~11925 counts
        #
        # See docs/CONTEXT.md section 5.
        RUN.advance()
        await asyncio.sleep_ms(STEP_PAUSE_MS)
        # ^^^^ END BLOCK ^^^^

        await asyncio.sleep(0)


# ------------------------------------------------------------------ web server
def _read_file(name):
    try:
        with open(name, "rb") as f:
            return f.read()
    except OSError:
        return None


async def _send(writer, status, ctype, body):
    if isinstance(body, str):
        body = body.encode()
    head = ("HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
            "Cache-Control: no-store\r\nConnection: close\r\n\r\n"
            % (status, ctype, len(body)))
    writer.write(head.encode())
    writer.write(body)
    await writer.drain()


async def _json(writer, obj, status="200 OK"):
    await _send(writer, status, "application/json", ujson.dumps(obj))


def _query(q):
    out = {}
    for kv in q.split("&"):
        if not kv:
            continue
        k, _, v = kv.partition("=")
        out[k] = v
    return out


def _parse_cell(text):
    """'2,1' -> (2, 1); None if malformed or off-grid."""
    try:
        r, c = [int(x) for x in text.split(",")]
    except (ValueError, AttributeError):
        return None
    if 0 <= r < config.GRID and 0 <= c < config.GRID:
        return (r, c)
    return None


async def handle(reader, writer):
    try:
        line = await reader.readline()
        if not line:
            return
        parts = line.decode().split()
        if len(parts) < 2:
            return
        target = parts[1]
        while True:                                  # drain headers
            h = await reader.readline()
            if not h or h == b"\r\n":
                break

        path, _, query = target.partition("?")
        args = _query(query)

        if path == "/":
            page = _read_file("index.html")
            if page is None:
                await _send(writer, "500 Internal Server Error", "text/plain",
                            "index.html missing on the Pico filesystem")
            else:
                await _send(writer, "200 OK", "text/html; charset=utf-8", page)

        elif path == "/state":
            await _json(writer, RUN.state)

        elif path == "/select":
            ok, why = RUN.select(args.get("algo", ""))
            # 409 on a mid-run change, so the dashboard can resync rather than
            # quietly disagreeing with the robot about what is executing.
            await _json(writer, {"ok": ok, "algo": RUN.state["algo"], "error": None if ok else why},
                        "200 OK" if ok else "409 Conflict")

        elif path == "/run":
            start = _parse_cell(args["start"]) if "start" in args else None
            ok, why = RUN.run(start=start, heading=args.get("heading"))
            await _json(writer, {"ok": ok, "algo": RUN.state["active_algo"],
                                 "error": None if ok else why},
                        "200 OK" if ok else "409 Conflict")

        elif path == "/reset":
            RUN.stop()
            await _json(writer, {"ok": True})

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


# ------------------------------------------------------------------------ wifi
def connect_wifi(timeout_s=20):
    import time
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
    await asyncio.start_server(handle, "0.0.0.0", 80)
    print("server up, algorithms:", ALGOS)
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        asyncio.new_event_loop()
