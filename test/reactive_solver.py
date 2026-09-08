"""
reactive_solver.py — TRUE LEFT-HAND RULE maze solver, reporting to the website

No mapping, no position tracking ON THE ROBOT. Follows the left wall
religiously, which is GUARANTEED to reach the exit in a simply-connected maze
(all walls connected to the outer boundary — no free-standing island walls).

Logic each step (left-first priority = the classic left-hand rule):
    read front / left / right
    if LEFT is open       -> turn left,  then forward one cell
    elif FRONT is open    -> go forward one cell
    elif RIGHT is open    -> turn right, then forward one cell
    else (dead end)       -> turn around, then forward one cell

"Always try to turn left; keep your left hand on the wall." Following one
wall continuously traces a path from entrance to exit.

WHAT IS ADDED HERE, and nothing else — the motion code below is untouched:
  * wifi
  * waits for START to be pressed on the website, instead of running on boot
  * reports every action to the hub, so the maze on the website moves because
    the ROBOT moved
  * the WEBSITE does the position bookkeeping. The robot still knows nothing
    but "left is open, so I turn left" — the server turns that stream of
    turns and forwards into a dot on a map, and tells the robot to stop when
    that dot reaches the goal cell.

    http://<HUB_HOST>:8787/maze_runner   <- watch it here
    http://<HUB_HOST>:8787/runs          <- history, afterwards

Uses YOUR tuned/verified values:
    forward one cell = CELL_CM (19 cm by default) -> 8092 counts
    90 turn          = 2645 counts
Pinout: front Echo on GP17.
"""

from machine import Pin, PWM, time_pulse_us
import time
import network

try:
    import urequests as requests
except ImportError:
    import requests

# ---- EDIT THESE ----
WIFI_SSID = "your-wifi-name"
WIFI_PASS = "your-wifi-password"
# --------------------

# The hub is reachable more than one way. The robot tries these IN ORDER at
# startup and uses the first that answers, so moving between wifi networks or
# losing a tunnel does not mean editing code on the Pico.
#
# Plain http:// everywhere on purpose: MicroPython's urequests does not follow
# the 307 redirect an https-only endpoint sends, and its TLS stack is
# unreliable. Never put an https:// URL in this list.
CONTROL_URLS = [
    "http://<HUB_HOST>:8787",   # wherever tools/telemetry_receiver.py is running
    # add a second URL here (e.g. a tunnel) as a fallback if you have one -
    # the robot tries each in order and uses the first that answers.
]
CONTROL_URL = CONTROL_URLS[0]             # pick_url() replaces this at boot
HTTP_TIMEOUT = 8

# ===== your tuned values =====
# MM_PER_COUNT is the measured wheel travel per encoder count. Everything else
# in millimetres is derived from it, so you change DISTANCES here in cm and
# never have to work out counts by hand.
MM_PER_COUNT = 0.02348       # measured: 138.23 mm wheel circumference / 5887 CPR

# ONE CELL: centre of a cell to centre of the next one, in YOUR maze.
# Measure it with a ruler down a straight corridor - do not guess it.
#   19 cm -> 8092 counts      23 cm -> 9796 counts
#   20 cm -> 8518 counts      28 cm -> 11925 counts
CELL_CM = 19.0
COUNTS_PER_CELL = round(CELL_CM * 10 / MM_PER_COUNT)

# 90 DEGREE TURN, tuned by hand on the floor. 2645 counts = 62.1 mm of travel
# per wheel, which implies a 79 mm wheelbase.
# If it consistently under-turns, scale it: measured 85 deg instead of 90 ->
#   TURN_COUNTS = round(2645 * 90 / 85) = 2800
TURN_COUNTS = 2645
# =============================

# ===== wall detection =====
WALL_THRESH_CM = 10.0        # closer than this = wall (blocked); farther = open

# HC-SR04 needs the previous echo to die away before the next ping, or ping 2
# hears ping 1 bouncing off a side wall. The datasheet says 60 ms between
# measurements; 6 ms was short enough to read ghosts in a small hard-walled
# maze. 3 samples at 60 ms costs ~180 ms per sensor - drop to 2 if the run
# feels slow, do not drop the gap.
PING_SAMPLES = 3
PING_GAP_MS  = 60
# ==========================

# ===== drift correction =====
# Set False to run your original uncorrected motion, for an A/B in the maze.
USE_CORRECTION = True

# Encoder signs: which way each encoder counts when that wheel drives the robot
# FORWARDS. From your pico_motor_dir_test.py bench run (left fwd = NEGATIVE).
# Only used for correction - the plain move/turn code still uses abs().
LEFT_ENC_SIGN  = -1
RIGHT_ENC_SIGN = 1

FRONT_STOP_MM  = 60          # front sonar reading when centred in a cell with a
                             # wall ahead. MEASURE IT: pico_cell_calibrate_test.py
FRONT_TOL_MM   = 4           # this close to right: leave it alone
MAX_NUDGE_MM   = 30          # refuse a bigger correction, it means a bad reading
NUDGE_DUTY     = 18000
TURN_BAL_KP    = 40          # keeps the two wheels of a spin travelling together
CREEP_DEADBAND_MM = 2.5      # spin translation we do not bother correcting
# ============================

# ===== how long to run =====
MAX_STEPS = 40               # stop after this many moves (safety)
PAUSE_S   = 0.4              # pause between moves
# ===========================

BASE_SPEED = 30000
TURN_DUTY  = 24000
KP = 400
TIMEOUT_MS = 8000

# ---------- sensors ----------
LEFT_S  = (Pin(11, Pin.OUT), Pin(12, Pin.IN))
FRONT_S = (Pin(13, Pin.OUT), Pin(17, Pin.IN))
RIGHT_S = (Pin(15, Pin.OUT), Pin(16, Pin.IN))
for trig, _ in (LEFT_S, FRONT_S, RIGHT_S):
    trig.value(0)
time.sleep_ms(50)

def _ping(sensor):
    trig, echo = sensor
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10); trig.value(0)
    dur = time_pulse_us(echo, 1, 30000)
    return -1 if dur < 0 else (dur * 0.0343) / 2

def read(sensor, n=None):
    vals = []
    for _ in range(n or PING_SAMPLES):
        d = _ping(sensor)
        if d > 0: vals.append(d)
        time.sleep_ms(PING_GAP_MS)
    if not vals: return -1
    vals.sort()
    return vals[len(vals)//2]

def is_open(dist):
    """open if no echo (-1) or farther than threshold"""
    return dist < 0 or dist >= WALL_THRESH_CM

# ---------- encoders ----------
class QuadEncoder:
    _TABLE = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)
    def __init__(self, pin_a, pin_b):
        self._a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.count = 0
        self._state = (self._a.value() << 1) | self._b.value()
        self._a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)
        self._b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._cb)
    def _cb(self, pin):
        new = (self._a.value() << 1) | self._b.value()
        self.count += self._TABLE[(self._state << 2) | new]
        self._state = new
    def reset(self):
        self.count = 0

left_enc  = QuadEncoder(7, 8)
right_enc = QuadEncoder(9, 10)

# ---------- motors ----------
STBY = Pin(6, Pin.OUT)
PWMA = PWM(Pin(0)); PWMA.freq(20000)
AIN1 = Pin(1, Pin.OUT); AIN2 = Pin(2, Pin.OUT)
PWMB = PWM(Pin(5)); PWMB.freq(20000)
BIN1 = Pin(3, Pin.OUT); BIN2 = Pin(4, Pin.OUT)

def lw(fwd, duty):
    if fwd: AIN1.value(0); AIN2.value(1)
    else:   AIN1.value(1); AIN2.value(0)
    PWMA.duty_u16(duty)
def rw(fwd, duty):
    if fwd: BIN1.value(1); BIN2.value(0)
    else:   BIN1.value(0); BIN2.value(1)
    PWMB.duty_u16(duty)
def drive(dl, dr):
    STBY.value(1)
    AIN1.value(0); AIN2.value(1)
    BIN1.value(1); BIN2.value(0)
    PWMA.duty_u16(max(0,min(65535,int(dl))))
    PWMB.duty_u16(max(0,min(65535,int(dr))))
def stop():
    PWMA.duty_u16(0); PWMB.duty_u16(0)
    AIN1.value(0); AIN2.value(0); BIN1.value(0); BIN2.value(0)
    STBY.value(0)
def clamp(v,lo,hi): return lo if v<lo else hi if v>hi else v

# ---------- correction helpers ----------
def enc_signed():
    """Encoder counts as FORWARD-POSITIVE for both wheels.

    The plain motion code uses abs(), which is fine for "have I gone far
    enough" but throws away the one thing that makes drift measurable: in a
    pure spin the two wheels turn OPPOSITE ways, so their signed sum is
    exactly the forward creep. abs() hides it.
    """
    return LEFT_ENC_SIGN * left_enc.count, RIGHT_ENC_SIGN * right_enc.count


def nudge_mm(dist_mm):
    """Creep forward (+) or back (-) a few mm. Every correction goes through
    here, and it is bounded so a bad sonar reading cannot drive into a wall."""
    if abs(dist_mm) < 0.5:
        return 0.0
    want = max(-MAX_NUDGE_MM, min(MAX_NUDGE_MM, dist_mm))
    target = abs(want) / MM_PER_COUNT
    left_enc.reset(); right_enc.reset()
    STBY.value(1)
    fwd = want > 0
    lw(fwd, NUDGE_DUTY); rw(fwd, NUDGE_DUTY)
    t0 = time.ticks_ms()
    while (abs(left_enc.count) + abs(right_enc.count)) / 2 < target:
        if time.ticks_diff(time.ticks_ms(), t0) > 2500:
            break
        time.sleep_ms(2)
    stop()
    moved = (abs(left_enc.count) + abs(right_enc.count)) / 2 * MM_PER_COUNT
    return moved if fwd else -moved


def settle_front():
    """Square up to the wall ahead, if there is one.

    This is the whole answer to accumulating distance error, and it needs no
    position tracking at all - which is why it suits a reactive solver. Any
    cell with a wall in front resets the robot to a known standoff, so a 3 mm
    overshoot now cannot become a 30 mm crash eight cells later.
    """
    if not USE_CORRECTION:
        return None
    f_cm = read(FRONT_S)
    if f_cm < 0 or f_cm >= WALL_THRESH_CM:
        return None                              # no wall to measure against
    err = f_cm * 10 - FRONT_STOP_MM              # +ve = stopped too far back
    if abs(err) <= FRONT_TOL_MM or abs(err) > MAX_NUDGE_MM:
        return None
    return nudge_mm(err)


# ---------- primitives ----------
def forward_one_cell():
    """Returns True if it actually travelled a cell.

    The original swallowed the timeout and returned as if the move had
    happened - so a stalled wheel silently advanced the map on the website.
    A move that did not happen must say so.
    """
    left_enc.reset(); right_enc.reset()
    t0 = time.ticks_ms()
    ok = True
    while True:
        l = abs(left_enc.count); r = abs(right_enc.count)
        if (l + r) / 2 >= COUNTS_PER_CELL: break
        if time.ticks_diff(time.ticks_ms(), t0) > TIMEOUT_MS:
            ok = False
            break
        err = l - r
        adj = KP * err // 100
        drive(clamp(BASE_SPEED-adj,0,65535), clamp(BASE_SPEED+adj,0,65535))
        time.sleep_ms(3)
    stop()
    if not ok:
        print("  !! forward timed out after %.0f mm - wheel stalled?"
              % ((abs(left_enc.count) + abs(right_enc.count)) / 2 * MM_PER_COUNT))
    return ok

def spin(counts, direction):
    """Spin in place. Returns the forward creep in mm, after correcting it.

    What was wrong with stopping on the AVERAGE of both wheels: a wheel that
    slips contributes nothing while the other keeps counting, so the average
    still reaches target - the robot ends up under-rotated AND pushed forward.
    Each wheel now stops on its own count, and a balance term eases off
    whichever wheel is ahead so the pivot stays on the axle centre.
    """
    left_enc.reset(); right_enc.reset()
    STBY.value(1)
    left_fwd = (direction == "right")      # right turn = left wheel forwards
    t0 = time.ticks_ms()
    while True:
        l = abs(left_enc.count); r = abs(right_enc.count)
        l_done, r_done = l >= counts, r >= counts
        if l_done and r_done:
            break
        if time.ticks_diff(time.ticks_ms(), t0) > TIMEOUT_MS:
            break
        if USE_CORRECTION:
            bal = TURN_BAL_KP * (l - r) // 100
            dl = 0 if l_done else clamp(TURN_DUTY - bal, 0, TURN_DUTY)
            dr = 0 if r_done else clamp(TURN_DUTY + bal, 0, TURN_DUTY)
        else:
            dl = dr = TURN_DUTY            # original behaviour
        lw(left_fwd, int(dl)); rw(not left_fwd, int(dr))
        time.sleep_ms(2)
    stop()

    if not USE_CORRECTION:
        return 0.0

    # signed sum = net translation. Only trust it if the wheels really did
    # counter-rotate; if they did not, the sign constants are wrong or a wheel
    # stalled, and "correcting" would make things worse.
    ls, rs = enc_signed()
    if ls * rs >= 0:
        print("  !! spin: wheels did not counter-rotate (l=%d r=%d)"
              " - check LEFT_ENC_SIGN/RIGHT_ENC_SIGN. No trim applied." % (ls, rs))
        return 0.0
    creep = (ls + rs) / 2 * MM_PER_COUNT
    if abs(creep) > CREEP_DEADBAND_MM:
        nudge_mm(-creep)
    return creep

def turn_left():   return spin(TURN_COUNTS, "left")
def turn_right():  return spin(TURN_COUNTS, "right")
def turn_around(): return spin(TURN_COUNTS * 2, "left")

# ================= website =================
# Everything below is the network layer. It never decides anything about
# where to drive - it only reports what the solver above already did.

def connect_wifi(timeout_s=25):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("connecting to", WIFI_SSID, "...")
        wlan.connect(WIFI_SSID, WIFI_PASS)
        t0 = time.time()
        while not wlan.isconnected():
            if time.time() - t0 > timeout_s:
                raise RuntimeError("wifi connect timed out - check SSID/pass")
            time.sleep(0.5)
    print("connected, ip:", wlan.ifconfig()[0])
    return wlan


def post(path, payload):
    try:
        r = requests.post(CONTROL_URL + path, json=payload, timeout=HTTP_TIMEOUT)
        r.close()
        return True
    except Exception as e:
        print("post", path, "failed:", e)
        return False


def get(path):
    try:
        r = requests.get(CONTROL_URL + path, timeout=HTTP_TIMEOUT)
        j = r.json()
        r.close()
        return j
    except Exception as e:
        print("get", path, "failed:", e)
        return None


def _turn_note(word, creep_mm):
    """The turn, plus how far it pushed us forward and what we did about it."""
    if not USE_CORRECTION or not creep_mm:
        return word
    return "%s [crept %+.1fmm, trimmed back]" % (word, creep_mm)


def mm(cm):
    """cm from read() -> mm for the website. -1 (no echo) means open space."""
    return 4000 if cm < 0 else round(cm * 10)


def send_event(kind, note=None, dists=None, blocked=None):
    """Tell the website what just happened.

    No pos, no heading - this robot does not know either. The hub keeps the
    dead reckoning and draws the map from these events alone.
    """
    body = {"kind": kind}
    if note is not None:
        body["note"] = note
    if dists is not None:
        body["sonar"] = {"L": mm(dists[0]), "F": mm(dists[1]), "R": mm(dists[2])}
    if blocked is not None:
        body["rel_walls"] = blocked      # any of "L","F","R" that are walls
    post("/event", body)


def pick_url():
    """Find a hub that answers, and use it. Called once at startup.

    Prints exactly what is wrong rather than failing silently: a robot that
    cannot reach the hub looks identical to a robot that will not start.
    """
    global CONTROL_URL
    for u in CONTROL_URLS:
        try:
            r = requests.get(u + "/cmd-state", timeout=HTTP_TIMEOUT)
            code = r.status_code
            r.close()
        except Exception as e:
            print("  x", u, "-", e)
            continue
        if code == 200:
            CONTROL_URL = u
            print("  link ok:", u)
            return True
        if code in (301, 302, 303, 307, 308):
            print("  x", u, "redirected (HTTP %d) - urequests will not follow"
                  " a redirect. Use a plain-http endpoint." % code)
        else:
            print("  x", u, "answered HTTP", code)
    print("!! no hub reachable. Check: wifi connected? server up?")
    print("!! tried:", ", ".join(CONTROL_URLS))
    return False


def pending_stop():
    """The website says stop - either you pressed Stop, or the hub's map says
    the robot has arrived at the goal cell. Checked between moves."""
    j = get("/cmd-state")
    if not j:
        return None
    c = j.get("cmd")
    if c and c.get("cmd") == "STOP":
        return c
    return None


# ---------- reactive loop ----------
def solve():
    for step in range(1, MAX_STEPS + 1):
        c = pending_stop()
        if c:
            # the hub says stop, and says why: "goal" when its map shows we
            # arrived, "stopped" when someone pressed Stop on the page
            why = c.get("reason") or "stopped"
            print("website says stop (%s)" % why)
            post("/cmd-done", {"seq": c.get("seq"), "cmd": "STOP", "ok": True})
            return why

        l = read(LEFT_S); f = read(FRONT_S); r = read(RIGHT_S)
        def s(d): return "--" if d < 0 else "%.0f" % d
        print("step %d | L %s F %s R %s ->" % (step, s(l), s(f), s(r)), end=" ")

        blocked = []
        if not is_open(l): blocked.append("L")
        if not is_open(f): blocked.append("F")
        if not is_open(r): blocked.append("R")
        send_event("sense", dists=(l, f, r), blocked=blocked)

        if is_open(l):
            print("turn left + forward")
            creep = turn_left()
            send_event("turn", note=_turn_note("left", creep))
            time.sleep(PAUSE_S)
        elif is_open(f):
            print("forward")
        elif is_open(r):
            print("turn right + forward")
            creep = turn_right()
            send_event("turn", note=_turn_note("right", creep))
            time.sleep(PAUSE_S)
        else:
            print("dead end -> turn around + forward")
            creep = turn_around()
            send_event("turn", note=_turn_note("around", creep))
            time.sleep(PAUSE_S)

        if not forward_one_cell():
            send_event("stuck", note="forward timed out - wheel stalled or blocked")
            return "stuck"
        trim = settle_front()
        send_event("forward", note=(None if trim is None
                                    else "squared up to front wall %+.1fmm" % trim))

        time.sleep(PAUSE_S)

    print("reached MAX_STEPS (%d). stopping." % MAX_STEPS)
    send_event("stuck", note="hit the %d step safety limit" % MAX_STEPS)
    return "stuck"


def main():
    print("reactive solver — waiting for START on the website")
    connect_wifi()
    print("looking for the hub...")
    pick_url()
    print("watch:", CONTROL_URL + "/maze_runner")

    while True:
        j = get("/cmd-state")
        if j:
            c = j.get("cmd")
            if c and c.get("cmd") == "MAZE":
                print("START received (seq %s) - running in 2 s" % c.get("seq"))
                time.sleep(2)
                try:
                    result = solve()
                except Exception as e:
                    stop()
                    result = "error"
                    print("run failed:", e)
                    send_event("stuck", note="firmware error: %s" % e)
                stop()
                print("run finished:", result)
                post("/cmd-done", {"seq": c.get("seq"), "cmd": "MAZE",
                                   "ok": result == "goal",
                                   "report": {"result": result}})
            else:
                # idle: keep the website's "clear ahead / wall ahead" banner
                # honest by pinging the sonars every couple of seconds
                l = read(LEFT_S, 3); f = read(FRONT_S, 3); r = read(RIGHT_S, 3)
                post("/status", {"src": "pico", "version": "reactive_solver",
                                 "sonar_mm": {"L": mm(l), "F": mm(f), "R": mm(r)}})
        time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        stop()
        print("stopped.")
