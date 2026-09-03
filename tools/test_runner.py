#!/usr/bin/env python3
"""Test the run lifecycle: latching, mid-run lock, reset, arbitrary starts.

Run from the repo root:  python3 tools/test_runner.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pico"))

from runner import Runner                      # noqa: E402
from config import GRID, GOAL                  # noqa: E402

CELLS = [(r, c) for r in range(GRID) for c in range(GRID)]
fails = []


def check(name, cond, detail=""):
    print("  %-52s %s" % (name, "ok" if cond else "FAIL " + detail))
    if not cond:
        fails.append(name)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        self.t += 0.8       # pretend each cell takes 0.8 s
        return self.t


print("SELECTION LOCK")
r = Runner(now=Clock())
ok, _ = r.select("astar")
check("can select while idle", ok and r.state["algo"] == "astar")
ok, _ = r.select("nonsense")
check("unknown algo rejected", not ok)
r.run()
check("active_algo latched at run", r.state["active_algo"] == "astar")
ok, why = r.select("lefthand")
check("select refused mid-run", not ok and why == "run in progress")
check("selection unchanged mid-run", r.state["algo"] == "astar")
ok, _ = r.run()
check("second run() refused mid-run", not ok)
mid = list(r.state["pos"])
r.advance()
check("robot advanced past its start", r.state["pos"] != mid)
r.stop()
check("stop clears running", not r.state["running"])
check("stop clears active_algo", r.state["active_algo"] is None)
ok, _ = r.select("lefthand")
check("select allowed again after stop", ok)

print("\nALGO CANNOT CHANGE MID-PATH")
r = Runner(now=Clock())
r.select("floodfill")
r.run()
r.advance()
r.state["algo"] = "lefthand"          # simulate a rogue write bypassing select()
path = []
while r.advance():
    path.append(tuple(r.state["pos"]))
check("run completed", r.solver.done)
check("executed the latched algo, not the rogue one", r.solver.algo == "floodfill")
check("took the optimal 10 steps", r.solver.steps == 10, str(r.solver.steps))

print("\nRUNS FROM EVERY START CELL")
for algo in ("floodfill", "lefthand", "astar"):
    bad = []
    for start in CELLS:
        r = Runner(now=Clock())
        r.select(algo)
        r.run(start=start)
        guard = 0
        while r.advance():
            guard += 1
            if guard > 200:
                break
        if not r.solver.done or tuple(r.state["pos"]) != tuple(GOAL):
            bad.append(start)
    check("%s reaches goal from all 16 cells" % algo, not bad, str(bad))

print("\nTELEMETRY SANITY")
r = Runner(now=Clock())
r.select("floodfill")
r.run(start=(3, 0))
while r.advance():
    pass
check("explored == len(discovered)", r.state["explored"] == len(r.state["discovered"]))
check("discovered has no duplicates",
      len(set(r.state["discovered"])) == len(r.state["discovered"]))
check("ended on the goal", tuple(r.state["pos"]) == tuple(GOAL))
check("running cleared at goal", not r.state["running"])
check("elapsed advanced", r.state["elapsed"] > 0)
check("message reports arrival", "Goal" in r.state["message"], r.state["message"])

print("\nSENSING -> ABSOLUTE WALLS")
r = Runner(now=Clock())
r.run(start=(3, 0))
r.state["heading"] = "S"
sides = r.note_walls({"front": True, "left": False, "right": True})
check("front+right while facing S -> S and W", sorted(sides) == ["S", "W"], str(sides))
check("message reports the front wall", r.state["message"] == "wall in front")
check("sensed recorded against the cell", r.state["sensed"].get("3,0") == sides)
r.state["heading"] = "E"
sides = r.note_walls({"front": False, "left": True, "right": False})
check("left while facing E -> N", sides == ["N"], str(sides))
check("message reports the left wall", r.state["message"] == "wall on left")
sides = r.note_walls({"front": False, "left": False, "right": False})
check("no walls -> clear ahead", sides == [] and r.state["message"] == "clear ahead")

print("\nFAILURE PATH")
r = Runner(now=Clock())
r.run()
r.fail("MotionTimeout: stalled")
check("fail clears running", not r.state["running"])
check("fail clears active_algo", r.state["active_algo"] is None)
check("fail records the reason", r.state["error"] == "MotionTimeout: stalled")
ok, _ = r.select("astar")
check("can select again after a failure", ok)

print("\nTURN GEOMETRY")
import maze as _maze
pairs = [("N", "N", "forward"), ("N", "E", "turn_right"), ("N", "W", "turn_left"),
         ("N", "S", "turn_around"), ("E", "S", "turn_right"), ("W", "N", "turn_right")]
bad = [(h, t, _maze.action_for_turn(h, t)) for h, t, want in pairs
       if _maze.action_for_turn(h, t) != want]
check("action_for_turn correct on all sampled pairs", not bad, str(bad))

print("\n%s" % ("all passed" if not fails else "%d FAILED: %s" % (len(fails), fails)))
sys.exit(1 if fails else 0)
