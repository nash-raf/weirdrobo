# Known/pre-mapped 4x4 maze + the three solving algorithms.
# Coordinates: (row, col), row 0 = bottom, row 3 = top, col 0 = left, col 3 = right.

from config import GRID, START, GOAL

# Transcribed physical maze. Symmetric (adjacent cells agree), border walls included.
# CONFIRM against the physical maze before a run - this may be re-edited.
WALLS = {
    "0,0": ["E", "S", "W"], "0,1": ["S", "W"], "0,2": ["E", "S"], "0,3": ["E", "S", "W"],
    "1,0": ["W"],           "1,1": ["E", "N"], "1,2": ["W"],      "1,3": ["E", "N"],
    "2,0": ["N", "W"],      "2,1": ["E", "S"], "2,2": ["E", "W"], "2,3": ["E", "S", "W"],
    "3,0": ["N", "S", "W"], "3,1": ["E", "N"], "3,2": ["N", "W"], "3,3": ["E", "N"],
}

# Heading -> (d_row, d_col). North is towards row 3 (top of the drawn map).
DELTA = {"N": (1, 0), "S": (-1, 0), "E": (0, 1), "W": (0, -1)}
RIGHT_OF = {"N": "E", "E": "S", "S": "W", "W": "N"}
LEFT_OF = {"N": "W", "W": "S", "S": "E", "E": "N"}
BACK_OF = {"N": "S", "S": "N", "E": "W", "W": "E"}


def key(r, c):
    return "%d,%d" % (r, c)


def in_bounds(r, c):
    return 0 <= r < GRID and 0 <= c < GRID


def has_wall(r, c, side):
    return side in WALLS.get(key(r, c), [])


def neighbours(r, c):
    """Cells reachable from (r, c) - in bounds and no wall between."""
    out = []
    for side, (dr, dc) in DELTA.items():
        if has_wall(r, c, side):
            continue
        nr, nc = r + dr, c + dc
        if in_bounds(nr, nc):
            out.append((nr, nc, side))
    return out


# ---------------------------------------------------------------- flood fill
def flood_fill(goal=GOAL):
    """Distance-to-goal for every cell. BFS outward from the goal."""
    dist = {}
    dist[goal] = 0
    frontier = [goal]
    while frontier:
        nxt = []
        for (r, c) in frontier:
            for (nr, nc, _side) in neighbours(r, c):
                if (nr, nc) not in dist:
                    dist[(nr, nc)] = dist[(r, c)] + 1
                    nxt.append((nr, nc))
        frontier = nxt
    return dist


def step_floodfill(pos, heading, goal=GOAL):
    """Next cell: the reachable neighbour with the lowest distance-to-goal."""
    dist = flood_fill(goal)
    best = None
    for (nr, nc, side) in neighbours(*pos):
        d = dist.get((nr, nc))
        if d is None:
            continue
        if best is None or d < best[0]:
            best = (d, (nr, nc), side)
    return (best[1], best[2]) if best else (pos, heading)


# ------------------------------------------------------------ left-hand rule
def step_lefthand(pos, heading, goal=GOAL):
    """Wall follower: try left, then straight, then right, then turn around."""
    r, c = pos
    for side in (LEFT_OF[heading], heading, RIGHT_OF[heading], BACK_OF[heading]):
        if has_wall(r, c, side):
            continue
        dr, dc = DELTA[side]
        if in_bounds(r + dr, c + dc):
            return ((r + dr, c + dc), side)
    return (pos, heading)


# ------------------------------------------------------------------------ A*
def _manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar_path(start=START, goal=GOAL):
    """Full path start -> goal as a list of cells. Manhattan heuristic."""
    open_set = [start]
    came = {}
    g = {start: 0}
    f = {start: _manhattan(start, goal)}
    while open_set:
        cur = min(open_set, key=lambda n: f.get(n, 1 << 30))
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        open_set.remove(cur)
        for (nr, nc, _side) in neighbours(*cur):
            tentative = g[cur] + 1
            if tentative < g.get((nr, nc), 1 << 30):
                came[(nr, nc)] = cur
                g[(nr, nc)] = tentative
                f[(nr, nc)] = tentative + _manhattan((nr, nc), goal)
                if (nr, nc) not in open_set:
                    open_set.append((nr, nc))
    return [start]


def step_astar(pos, heading, goal=GOAL):
    path = astar_path(pos, goal)
    if len(path) < 2:
        return (pos, heading)
    nxt = path[1]
    dr, dc = nxt[0] - pos[0], nxt[1] - pos[1]
    for side, d in DELTA.items():
        if d == (dr, dc):
            return (nxt, side)
    return (pos, heading)


STEPPERS = {
    "floodfill": step_floodfill,
    "lefthand": step_lefthand,
    "astar": step_astar,
}


def next_move(algo, pos, heading, goal=GOAL):
    """Returns (next_cell, heading_to_face). Single entry point for robot_task()."""
    return STEPPERS.get(algo, step_floodfill)(pos, heading, goal)
