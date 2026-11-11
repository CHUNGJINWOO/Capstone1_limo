"""Pure grid checks. Inflation evidence is a hypothesis, never collision proof."""
from collections import deque
import math


def valid_grid(grid):
    return (grid.info.resolution > 0 and grid.info.width > 0
            and grid.info.height > 0
            and len(grid.data) == grid.info.width * grid.info.height)


def yaw(grid):
    q = grid.info.origin.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def index(grid, x, y):
    angle = yaw(grid)
    dx = x - grid.info.origin.position.x
    dy = y - grid.info.origin.position.y
    cx = math.floor((math.cos(angle) * dx + math.sin(angle) * dy)
                    / grid.info.resolution)
    cy = math.floor((-math.sin(angle) * dx + math.cos(angle) * dy)
                    / grid.info.resolution)
    if 0 <= cx < grid.info.width and 0 <= cy < grid.info.height:
        return cy * grid.info.width + cx
    return None


def world(grid, cell):
    x = (cell % grid.info.width + 0.5) * grid.info.resolution
    y = (cell // grid.info.width + 0.5) * grid.info.resolution
    a = yaw(grid)
    return (grid.info.origin.position.x + math.cos(a) * x - math.sin(a) * y,
            grid.info.origin.position.y + math.sin(a) * x + math.cos(a) * y)


def neighbors(cell, width, height):
    x, y = cell % width, cell // width
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        if 0 <= x + dx < width and 0 <= y + dy < height:
            yield (y + dy) * width + x + dx


def frontiers(grid, robot):
    """Free cells adjacent to unknown, connected by raw-map free cells.

    Connectivity deliberately does not claim footprint clearance or Nav2 reachability.
    """
    if not valid_grid(grid):
        raise ValueError('Invalid occupancy grid')
    start = index(grid, *robot)
    if start is None or grid.data[start] != 0:
        raise ValueError('Robot must be inside a known free map cell')
    seen = {start}
    queue = deque([start])
    result = []
    while queue:
        cell = queue.popleft()
        adjacent = list(neighbors(cell, grid.info.width, grid.info.height))
        if any(grid.data[n] == -1 for n in adjacent):
            result.append(world(grid, cell))
        for n in adjacent:
            if n not in seen and grid.data[n] == 0:
                seen.add(n)
                queue.append(n)
    return sorted(result, key=lambda xy: math.hypot(xy[0]-robot[0], xy[1]-robot[1]))


def cost(grid, point):
    i = index(grid, *point)
    return None if i is None else grid.data[i]


def suppressed_candidates(raw, inflated, robot):
    if not valid_grid(inflated) or raw.header.frame_id != inflated.header.frame_id:
        raise ValueError('Valid grids in the same frame are required')
    # OccupancyGrid translation: 99 is inscribed, 100 lethal, -1 unknown.
    # None of those may be treated as reducible inflation.
    return [p for p in frontiers(raw, robot)
            if (value := cost(inflated, p)) is not None and 0 < value < 99]


def validate_radii(original, reduced, robot_radius, minimum_clearance):
    values = [*original, reduced, robot_radius, minimum_clearance]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Radii must be finite')
    if robot_radius <= 0 or minimum_clearance <= 0:
        raise ValueError('Positive radius and clearance are required')
    if reduced < robot_radius + minimum_clearance - 1e-9:
        raise ValueError('Reduced radius below configured radius + clearance floor')
    if any(reduced >= radius for radius in original):
        raise ValueError('Reduced radius must be below BOTH original inflation radii')
