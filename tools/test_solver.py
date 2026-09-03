#!/usr/bin/env python3
"""Exercise the solver from every cell, in every algorithm, on the real maze.

Run from the repo root:  python3 tools/test_solver.py
No hardware, no MicroPython - pure logic, so it runs anywhere.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pico"))

import maze                                    # noqa: E402
from solver import Solver                      # noqa: E402
from config import GRID, START, GOAL           # noqa: E402

ALGOS = ["floodfill", "lefthand", "astar"]
HEADINGS = ["N", "S", "E", "W"]
CELLS = [(r, c) for r in range(GRID) for c in range(GRID)]


def shortest_len(start, goal=GOAL):
    """Ground truth: BFS hop count. -1 if unreachable."""
    if start == goal:
        return 0
    seen = {start}
    frontier = [start]
    d = 0
    while frontier:
        d += 1
        nxt = []
        for cell in frontier:
            for (nr, nc, _s) in maze.neighbours(*cell):
                if (nr, nc) == goal:
                    return d
                if (nr, nc) not in seen:
                    seen.add((nr, nc))
                    nxt.append((nr, nc))
        frontier = nxt
    return -1


def check_map():
    """The map must be symmetric and fully connected, or nothing below means much."""
    problems = []
    opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
    for (r, c) in CELLS:
        for side, (dr, dc) in maze.DELTA.items():
            nr, nc = r + dr, c + dc
            if not maze.in_bounds(nr, nc):
                if not maze.has_wall(r, c, side):
                    problems.append("(%d,%d) open to %s but that is off-grid" % (r, c, side))
                continue
            mine = maze.has_wall(r, c, side)
            theirs = maze.has_wall(nr, nc, opposite[side])
            if mine != theirs:
                problems.append("(%d,%d).%s=%s disagrees with (%d,%d).%s=%s"
                                % (r, c, side, mine, nr, nc, opposite[side], theirs))
    unreachable = [cell for cell in CELLS if cell != GOAL and shortest_len(cell) < 0]
    return problems, unreachable


def main():
    print("MAZE CHECK")
    problems, unreachable = check_map()
    for p in problems:
        print("  wall mismatch:", p)
    if unreachable:
        print("  unreachable from goal:", unreachable)
    if not problems and not unreachable:
        print("  symmetric, fully connected, all 16 cells can reach the goal")
    print()

    total = failed = 0
    summary = {}

    for algo in ALGOS:
        print("ALGO: %s" % algo)
        worst = 0
        algo_fail = 0
        for start in CELLS:
            opt = shortest_len(start)
            for heading in HEADINGS:
                total += 1
                s = Solver(algo, start=start, heading=heading)
                ok = s.solve()
                if not ok:
                    failed += 1
                    algo_fail += 1
                    print("  FAIL  start=%s heading=%s -> %s (%d steps)"
                          % (start, heading, s.failure, s.steps))
                    continue
                if s.pos != tuple(GOAL):
                    failed += 1
                    algo_fail += 1
                    print("  FAIL  start=%s heading=%s ended at %s" % (start, heading, s.pos))
                    continue
                worst = max(worst, s.steps)
                if algo in ("floodfill", "astar") and s.steps != opt:
                    failed += 1
                    algo_fail += 1
                    print("  FAIL  start=%s heading=%s took %d, optimal is %d"
                          % (start, heading, s.steps, opt))
        summary[algo] = (algo_fail, worst)
        print("  %d/%d runs reached the goal, longest %d steps"
              % (len(CELLS) * len(HEADINGS) - algo_fail, len(CELLS) * len(HEADINGS), worst))
        print()

    print("REFERENCE RUN  start=%s heading=S" % (START,))
    for algo in ALGOS:
        s = Solver(algo, start=START, heading="S")
        s.solve()
        print("  %-10s %2d steps  %s" % (algo, s.steps, " -> ".join("%d%d" % p for p in s.path)))
    print()

    print("%d/%d runs passed" % (total - failed, total))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
