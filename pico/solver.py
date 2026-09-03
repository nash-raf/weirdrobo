# High-level maze solving, one cell at a time.
#
# Deliberately has NO hardware in it. The robot loop calls step() once per
# physical cell move; the simulator calls it in a tight loop. Same code path,
# so anything proved here is proved for the robot.

import maze
from config import START, GOAL

MAX_STEPS = 64          # 16 cells; anything beyond this is a loop, not a solve

FAIL_NONE = None
FAIL_STUCK = "stuck"        # no legal move out of this cell
FAIL_LOOP = "loop"          # revisited an identical (pos, heading) - cycling
FAIL_LIMIT = "limit"        # hit MAX_STEPS


class Solver:
    """Drives one algorithm from `start` to `goal`, one cell per step().

    The algorithm is latched at construction. It cannot change mid-run - that
    is the point: a run committed to flood fill finishes as flood fill.
    """

    def __init__(self, algo, start=START, goal=GOAL, heading="S", max_steps=MAX_STEPS):
        if algo not in maze.STEPPERS:
            raise ValueError("unknown algo: %s" % algo)
        self.algo = algo                    # latched, read-only for the run
        self.goal = tuple(goal)
        self.pos = tuple(start)
        self.heading = heading
        self.max_steps = max_steps

        self.path = [self.pos]
        self.visited = [self.pos]
        self.steps = 0
        self.done = (self.pos == self.goal)
        self.failure = FAIL_NONE
        self.message = "Standing by."
        self._seen = set()                  # (pos, heading) states, for loop detection

    # -- reporting ---------------------------------------------------------
    def _sense(self):
        """What a wall-sensing robot would report from here, for the UI."""
        r, c = self.pos
        if maze.has_wall(r, c, self.heading):
            return "wall in front"
        if maze.has_wall(r, c, maze.LEFT_OF[self.heading]):
            return "wall on left"
        if maze.has_wall(r, c, maze.RIGHT_OF[self.heading]):
            return "wall on right"
        return "clear ahead"

    @property
    def finished(self):
        return self.done or self.failure is not None

    # -- the loop ----------------------------------------------------------
    def step(self):
        """Advance one cell. Returns True if the robot moved."""
        if self.finished:
            return False

        if self.pos == self.goal:
            self.done = True
            self.message = "Goal reached."
            return False

        if self.steps >= self.max_steps:
            self.failure = FAIL_LIMIT
            self.message = "No route found (step limit)."
            return False

        # A deterministic algorithm that returns to an identical (cell, heading)
        # will repeat forever. Catch it here rather than burning the step limit.
        state_key = (self.pos, self.heading)
        if state_key in self._seen:
            self.failure = FAIL_LOOP
            self.message = "No route found (cycling)."
            return False
        self._seen.add(state_key)

        nxt, heading = maze.next_move(self.algo, self.pos, self.heading, self.goal)
        if nxt == self.pos:
            self.failure = FAIL_STUCK
            self.message = "Stuck - no move available."
            return False

        self.message = self._sense()
        self.heading = heading
        self.pos = tuple(nxt)
        self.steps += 1
        self.path.append(self.pos)
        if self.pos not in self.visited:
            self.visited.append(self.pos)

        if self.pos == self.goal:
            self.done = True
            self.message = "Goal reached."
        return True

    def solve(self):
        """Run to completion. Only for the simulator - the robot uses step()."""
        while not self.finished:
            self.step()
        return self.done
