"""관측 경계의 기하 점수에 방향·실패 이력을 더한다. 주행 가능성 보장은 아니다."""
import math
from .policy import cost


def rank_candidates(candidates, robot, yaw, history, radius, minimum_distance,
                    heading_weight, failure_weight):
    ranked = []
    for candidate in candidates:
        dx, dy = candidate.point[0]-robot[0], candidate.point[1]-robot[1]
        distance = math.hypot(dx, dy)
        if distance < minimum_distance:
            continue  # Nav2 도착 공차 안의 가까운 점을 반복 목표로 보내지 않는다.
        angle = abs(math.atan2(math.sin(math.atan2(dy, dx)-yaw),
                               math.cos(math.atan2(dy, dx)-yaw)))
        failures = max((info.get('fails', 0) for point, info in history.items()
                        if math.hypot(candidate.point[0]-point[0],
                                      candidate.point[1]-point[1]) <= radius), default=0)
        score = candidate.score + heading_weight*angle + failure_weight*failures
        ranked.append((candidate, {'score': score, 'base_score': candidate.score,
                                  'heading_error_rad': angle, 'nearby_failures': failures,
                                  'distance_m': distance}))
    return sorted(ranked, key=lambda item: (item[1]['score'], item[0].point))


def checked_path(poses, point, grid):
    """경로 끝점·모든 점의 비용을 확인하고 마지막 유효 구간의 도착 방향을 반환한다."""
    if not poses:
        return False, None, 'empty_path'
    xy = [(p.pose.position.x, p.pose.position.y) for p in poses]
    if math.hypot(xy[-1][0]-point[0], xy[-1][1]-point[1]) > 1.5*grid.info.resolution:
        return False, None, 'path_stops_short'
    for position in xy:
        value = cost(grid, position)
        if value is None or not 0 <= value < 99:
            return False, None, 'path_unknown_or_inscribed'
    for before in reversed(xy[:-1]):
        if math.hypot(xy[-1][0]-before[0], xy[-1][1]-before[1]) > 1e-6:
            return True, math.atan2(xy[-1][1]-before[1], xy[-1][0]-before[0]), 'path_checked'
    return False, None, 'path_has_no_direction'
