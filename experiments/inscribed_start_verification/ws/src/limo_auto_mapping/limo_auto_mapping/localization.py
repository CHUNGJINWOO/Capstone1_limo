"""재위치 추정의 순수 검사 함수. 통과는 위치의 확정이나 충돌 안전 보장이 아니다."""
from dataclasses import dataclass
from collections import deque
import math

from .policy import cost, index, valid_grid


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def planar_pose(pose):
    """2D 자세만 허용하고 잘못된 quaternion이나 roll/pitch는 거부한다."""
    q = pose.orientation
    values = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite pose')
    if abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.0) > 0.01:
        raise ValueError('Invalid quaternion')
    roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
    pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
    if abs(roll) > .05 or abs(pitch) > .05:
        raise ValueError('Nonplanar pose/laser TF needs review')
    return (pose.position.x, pose.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))


def close_pose(a, b, distance=.20, angle=.15):
    return (math.hypot(a[0]-b[0], a[1]-b[1]) <= distance
            and abs(angle_difference(a[2], b[2])) <= angle)


def covariance_ok(values, xy_std, yaw_std):
    """상관항을 포함한 XY 최대 고윳값과 yaw 분산을 검사한다."""
    if len(values) != 36 or not all(math.isfinite(v) for v in values):
        return False
    a, b, c, d = values[0], values[1], values[6], values[7]
    if min(a, d, values[35]) < 0 or abs(b-c) > 1e-6 or a*d-b*c < -1e-9:
        return False
    largest = (a+d + math.sqrt((a-d)**2+4*b*b))/2
    return largest <= xy_std**2 and values[35] <= yaw_std**2


@dataclass
class Match:
    beams: int
    hit_ratio: float
    known_ratio: float
    blocked_ratio: float
    sectors: int


class MapMatcher:
    """scan 끝점의 벽 정합과 광선 중간의 알려진 장애물 통과 여부를 별도 검사한다."""
    def __init__(self, grid, wall_tolerance=.15, clearance=.24):
        if not valid_grid(grid) or not math.isfinite(grid.info.resolution):
            raise ValueError('Invalid localization map')
        planar_pose(grid.info.origin)
        self.grid = grid
        self.tolerance = wall_tolerance
        self.clearance = clearance
        radius = math.ceil(wall_tolerance/grid.info.resolution)
        self.offsets = [(x, y) for x in range(-radius, radius+1)
                        for y in range(-radius, radius+1)
                        if math.hypot(x, y)*grid.info.resolution <= wall_tolerance+1e-9]
        # 정지한 AMCL 입자의 작은 좌표 오차를 보정할 때만 쓰는 거리장이다.
        self.distances = [0.5]*len(grid.data)
        queue = deque()
        for i, value in enumerate(grid.data):
            if value >= 50:
                self.distances[i] = 0.
                queue.append(i)
        width, height = grid.info.width, grid.info.height
        while queue:
            i = queue.popleft()
            x, y = i % width, i // width
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                j = (y+dy)*width+x+dx
                distance = self.distances[i]+grid.info.resolution
                if 0 <= x+dx < width and 0 <= y+dy < height and distance < self.distances[j]:
                    self.distances[j] = distance
                    queue.append(j)

    def refine_pose(self, pose, scan, laser):
        """AMCL의 수렴 후보 근방 30cm/0.15rad 안에서 scan 끝점 정합을 보정한다.

        전역 위치 탐색을 대신하지 않는다. 보정 결과도 별도 광선·정합 검사를 통과해야 한다.
        """
        points = []
        for i in range(0, len(scan.ranges), max(1, len(scan.ranges)//60)):
            r = scan.ranges[i]
            if math.isfinite(r) and max(scan.range_min, .1) < r < scan.range_max:
                angle = laser[2]+scan.angle_min+i*scan.angle_increment
                points.append((laser[0]+r*math.cos(angle), laser[1]+r*math.sin(angle)))
        if len(points) < 15:
            return pose

        def loss(candidate):
            if not close_pose(pose, candidate, .30, .15):
                return float('inf')
            c, s = math.cos(candidate[2]), math.sin(candidate[2])
            total = 0.
            for x, y in points:
                cell = index(self.grid, candidate[0]+c*x-s*y, candidate[1]+s*x+c*y)
                total += self.distances[cell] if cell is not None else .5
            return total/len(points)

        best = pose
        value = loss(best)
        for step, rotation in ((.10, .05), (.05, .025), (.025, .0125)):
            for _ in range(8):
                options = [best]
                for axis, delta in ((0, step), (1, step), (2, rotation)):
                    for direction in (-1, 1):
                        candidate = list(best)
                        candidate[axis] += direction*delta
                        options.append(tuple(candidate))
                improved = min(options, key=loss)
                score = loss(improved)
                if score >= value-1e-6:
                    break
                best, value = improved, score
        return best

    def near_wall(self, x, y):
        cell = index(self.grid, x, y)
        if cell is None:
            return False
        width, height = self.grid.info.width, self.grid.info.height
        cx, cy = cell % width, cell // width
        return any(0 <= cx+dx < width and 0 <= cy+dy < height
                   and self.grid.data[(cy+dy)*width+cx+dx] >= 50
                   for dx, dy in self.offsets)

    def clear_center(self, pose):
        # 중심뿐 아니라 설정 반경 안의 알려진 free 셀도 확인한다.
        step = self.grid.info.resolution
        radius = math.ceil(self.clearance/step)
        return all(cost(self.grid, (pose[0]+dx*step, pose[1]+dy*step)) == 0
                   for dx in range(-radius, radius+1) for dy in range(-radius, radius+1)
                   if math.hypot(dx, dy)*step <= self.clearance+1e-9)

    def score(self, pose, scan, laser):
        """pose는 map→base, laser는 base→laser이다. 센서 장착 위치·방향을 반영한다."""
        if not self.clear_center(pose):
            return Match(0, 0., 0., 1., 0)
        if (not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment)
                or scan.angle_increment == 0 or not math.isfinite(scan.range_min)
                or not math.isfinite(scan.range_max) or scan.range_max <= scan.range_min):
            return Match(0, 0., 0., 1., 0)
        ct, st = math.cos(pose[2]), math.sin(pose[2])
        sx = pose[0]+ct*laser[0]-st*laser[1]
        sy = pose[1]+st*laser[0]+ct*laser[1]
        hits = blocked = known = total = count = 0
        sectors = set()
        stride = max(1, len(scan.ranges)//120)
        for i in range(0, len(scan.ranges), stride):
            r = scan.ranges[i]
            if not math.isfinite(r) or not max(scan.range_min, .10) < r < scan.range_max:
                continue
            relative = laser[2]+scan.angle_min+i*scan.angle_increment
            theta = pose[2]+relative
            dx, dy = math.cos(theta), math.sin(theta)
            count += 1
            hit = self.near_wall(sx+r*dx, sy+r*dy)
            hits += hit
            if hit:
                sectors.add(int((relative % (2*math.pi))/(math.pi/4)))
            ray_blocked = False
            # 끝점 부근은 지도 해상도/센서 오차만큼 허용하되 벽을 뚫는 경로는 거부한다.
            samples = max(1, math.ceil((r-self.tolerance)/max(.10, self.grid.info.resolution)))
            for j in range(samples):
                distance = j*(r-self.tolerance)/samples
                value = cost(self.grid, (sx+distance*dx, sy+distance*dy))
                total += 1
                known += value is not None and value >= 0
                ray_blocked |= value is not None and value >= 50
            blocked += ray_blocked
        return Match(count, hits/max(1, count), known/max(1, total),
                     blocked/max(1, count), len(sectors))


class StablePose:
    """새 scan 시각의 연속 관측만 인정한다. 반복된 과거 메시지는 성공을 만들지 않는다."""
    def __init__(self, seconds=5., samples=5, distance=.15, angle=.10):
        self.seconds, self.samples = seconds, samples
        self.distance, self.angle = distance, angle
        self.reset()

    def reset(self):
        self.history = []
        self.last_stamp = None

    def observe(self, now, stamp, pose, valid):
        if not valid:
            self.reset()
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.last_stamp = stamp
        if self.history and not all(close_pose(pose, p, self.distance, self.angle)
                                    for _, p in self.history):
            self.history = []
        self.history.append((now, pose))
        return (len(self.history) >= self.samples
                and now-self.history[0][0] >= self.seconds)
