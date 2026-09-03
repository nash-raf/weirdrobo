# Owns the shared `state` dict and the run lifecycle.
#
# No `machine`, `network` or `uasyncio` imports in here on purpose - this module
# runs unchanged on a desktop, so the whole run/lock/reset behaviour is testable
# without a Pico. main.py adds the async server on top; movement adds the motors.

import time

import maze
from config import START, GOAL
from solver import Solver

ALGOS = list(maze.STEPPERS.keys())


def _key(cell):
    return "%d,%d" % (cell[0], cell[1])


class Runner:
    def __init__(self, now=None):
        self._now = now or time.time
        self.solver = None
        self._t0 = 0.0
        self.state = {}
        self.reset(algo="floodfill")

    # -- state -------------------------------------------------------------
    def reset(self, algo=None, start=START, heading="S"):
        algo = algo or self.state.get("algo", "floodfill")
        self.solver = None
        self._t0 = 0.0
        self.state = {
            "pos": list(start),
            "heading": heading,
            "discovered": [_key(start)],
            "explored": 1,
            "elapsed": 0.0,
            "message": "Standing by.",
            "algo": algo,           # what the user has selected
            "active_algo": None,    # what is actually executing, latched at /run
            "running": False,
            "start": list(start),
            "goal": list(GOAL),
        }

    # -- commands ----------------------------------------------------------
    def select(self, algo):
        """Choose the algorithm. Refused mid-run: a run finishes as it started."""
        if algo not in ALGOS:
            return False, "unknown algo"
        if self.state["running"]:
            return False, "run in progress"
        self.state["algo"] = algo
        return True, algo

    def run(self, start=None, heading=None):
        """Start solving. The algorithm is latched here and cannot change."""
        if self.state["running"]:
            return False, "already running"
        start = tuple(start or self.state["start"])
        heading = heading or self.state["heading"]
        algo = self.state["algo"]

        self.reset(algo=algo, start=start, heading=heading)
        self.solver = Solver(algo, start=start, goal=GOAL, heading=heading)
        self._t0 = self._now()
        self.state["running"] = True
        self.state["active_algo"] = algo
        self.state["message"] = "Running %s." % algo
        return True, algo

    def stop(self):
        self.reset(algo=self.state["algo"], start=self.state["start"])

    # -- the loop ----------------------------------------------------------
    def advance(self):
        """One cell. Returns True while the run is still going."""
        if not self.state["running"] or self.solver is None:
            return False

        self.solver.step()
        s = self.solver

        self.state["pos"] = list(s.pos)
        self.state["heading"] = s.heading
        self.state["message"] = s.message
        self.state["elapsed"] = round(self._now() - self._t0, 1)

        k = _key(s.pos)
        if k not in self.state["discovered"]:
            self.state["discovered"].append(k)
            self.state["explored"] = len(self.state["discovered"])

        if s.finished:
            self.state["running"] = False
            self.state["active_algo"] = None
            return False
        return True
