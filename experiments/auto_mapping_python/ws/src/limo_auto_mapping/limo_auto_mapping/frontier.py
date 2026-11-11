"""Python 프론티어 선택: free 영역과 unknown 영역의 경계를 다음 관측 지점으로 쓴다.

m-explore-ros2의 frontier 탐색 개념을 참고한 독립 Python 구현이다.
C++ 탐색기의 정확한 포팅은 아니며 목표 선택·그룹화 결과가 달라질 수 있다.
참고 소스의 라이선스는 패키지의 UPSTREAM_LICENSE에 보관했다.
"""
from collections import deque
from dataclasses import dataclass
import math

from .policy import index, neighbors, valid_grid, world


@dataclass(frozen=True)
class Frontier:
    point: tuple       # Nav2에 보낼 지도 좌표 (x, y): unknown 셀이 아닌 free 셀이다.
    size: int          # 이 경계 그룹에 속한 free 셀 개수다.
    score: float       # 작을수록 가까우면서 관측할 경계가 많은 후보로 취급한다.


def adjacent8(cell, width, height):
    x, y = cell % width, cell // width
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if (dx or dy) and 0 <= x+dx < width and 0 <= y+dy < height:
                yield (y+dy)*width+x+dx


def choose_frontiers(grid, robot, minimum_size=0.2, potential_scale=3.0, gain_scale=1.0):
    """전체 global costmap에서 연결된 free 경계를 묶고 목표 후보를 정렬한다."""
    if not valid_grid(grid):
        raise ValueError('Invalid global costmap')
    start = index(grid, *robot)
    if start is None or not 0 <= grid.data[start] < 99:
        raise ValueError('Robot outside known traversable costmap')
    width, height = grid.info.width, grid.info.height

    # 로봇 중심이 낮은 inflation 비용 안에 있으면 연결된 영역에서 가까운 free 셀을 찾는다.
    # inscribed(99), lethal(100), unknown(-1)을 가로질러 free 영역을 찾지는 않는다.
    queue, seen = deque([start]), {start}
    seed = None
    while queue:
        cell = queue.popleft()
        if grid.data[cell] == 0:
            seed = cell
            break
        for n in neighbors(cell, width, height):
            if n not in seen and 0 <= grid.data[n] < 99:
                seen.add(n)
                queue.append(n)
    if seed is None:
        raise ValueError('No connected free cell around robot')

    # free 셀만 따라가며 미탐색(-1) 셀과 맞닿은 free 셀을 경계로 모은다.
    queue, seen, boundary = deque([seed]), {seed}, set()
    while queue:
        cell = queue.popleft()
        adjacent = tuple(neighbors(cell, width, height))
        if any(grid.data[n] == -1 for n in adjacent):
            boundary.add(cell)
        for n in adjacent:
            if n not in seen and grid.data[n] == 0:
                seen.add(n)
                queue.append(n)

    # 경계 셀을 8방향 인접 그룹으로 묶어 작은 노이즈 후보를 거른다.
    candidates = []
    while boundary:
        seed = min(boundary)  # 같은 입력 지도에서는 같은 결과가 나오도록 시작점을 고정한다.
        boundary.remove(seed)
        group, queue = [seed], deque([seed])
        while queue:
            for n in adjacent8(queue.popleft(), width, height):
                if n in boundary:
                    boundary.remove(n)
                    group.append(n)
                    queue.append(n)
        if len(group) * grid.info.resolution < minimum_size:
            continue
        points = [world(grid, cell) for cell in group]
        cx = sum(x for x, _ in points)/len(points)
        cy = sum(y for _, y in points)/len(points)
        # 평균 좌표 자체는 벽/unknown에 들어갈 수 있어 실제 free 셀 중 대표점을 고른다.
        target = min(points, key=lambda p: ((p[0]-cx)**2+(p[1]-cy)**2, p))
        distance = math.hypot(target[0]-robot[0], target[1]-robot[1])
        score = potential_scale*distance - gain_scale*len(group)*grid.info.resolution
        candidates.append(Frontier(target, len(group), score))
    return sorted(candidates, key=lambda candidate: (candidate.score, candidate.point))


def blacklisted(point, excluded, radius):
    # 실패 위치 주변을 다음 목표에서 잠시 제외한다. 새 탐색 회차에는 목록을 초기화한다.
    return any(math.hypot(point[0]-x, point[1]-y) <= radius for x, y in excluded)
