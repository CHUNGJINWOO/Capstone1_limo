"""교정된 평가 도구. map 좌표를 물리 접촉 판정에 사용하지 않는다."""
import math


def center_cost(grid, pose):
    """OccupancyGrid 원점 회전과 floor를 적용한다."""
    if grid is None or pose is None:
        return None
    info = grid.info
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = pose[0]-info.origin.position.x, pose[1]-info.origin.position.y
    x = math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution)
    y = math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution)
    if 0 <= x < info.width and 0 <= y < info.height:
        return int(grid.data[y*info.width+x])
    return None


def contact_episodes(samples, maximum_gap_sec=.15):
    """고유 timestamp·접촉쌍을 묶는다. 메시지 수를 충돌 횟수로 부르지 않는다."""
    grouped = {}
    for item in samples:
        pair = tuple(sorted(item['pair']))
        grouped.setdefault(pair, set()).add(item['stamp_ns'])
    result = []
    for pair, stamps in grouped.items():
        stamps = sorted(stamps)
        start = last = stamps[0]
        for stamp in stamps[1:]:
            if (stamp-last)/1e9 > maximum_gap_sec:
                result.append({'pair': list(pair), 'start_ns': start, 'end_ns': last,
                               'duration_sec': (last-start)/1e9})
                start = stamp
            last = stamp
        result.append({'pair': list(pair), 'start_ns': start, 'end_ns': last,
                       'duration_sec': (last-start)/1e9})
    return sorted(result, key=lambda x: x['start_ns'])


def trajectory_metrics(samples):
    """Gazebo 월드 pose의 서로 다른 timestamp만 사용한다."""
    unique = {s['stamp_ns']: s for s in samples}
    points = [unique[k] for k in sorted(unique)]
    if not points:
        return {'samples': 0, 'displacement_m': None, 'cumulative_distance_m': None}
    distance = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a, b in zip(points, points[1:]))
    return {'samples': len(points),
            'displacement_m': math.hypot(points[-1]['x']-points[0]['x'], points[-1]['y']-points[0]['y']),
            'cumulative_distance_m': distance, 'first': points[0], 'last': points[-1]}


def map_point_to_world(point, robot_map, robot_world):
    """시험 장애물 배치용 현재 pose 정합. planner에는 정답을 보내지 않는다."""
    yaw = robot_world[2]-robot_map[2]
    dx, dy = point[0]-robot_map[0], point[1]-robot_map[1]
    return (robot_world[0]+math.cos(yaw)*dx-math.sin(yaw)*dy,
            robot_world[1]+math.sin(yaw)*dx+math.cos(yaw)*dy)


def box_cost_summary(grid, center_world, size_world, robot_map, robot_world):
    """박스 내부·표면 셀을 함께 본다. 레이저는 박스 중심이 아닌 표면을 표시한다."""
    info = grid.info
    q = info.origin.orientation
    origin_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    yaw = origin_yaw + robot_world[2] - robot_map[2]
    origin_world = map_point_to_world((info.origin.position.x, info.origin.position.y),
                                      robot_map, robot_world)
    c, s = math.cos(yaw), math.sin(yaw)
    values = []
    for index, value in enumerate(grid.data):
        if value < 0:
            continue
        x = (index % info.width + .5)*info.resolution
        y = (index // info.width + .5)*info.resolution
        wx, wy = origin_world[0]+c*x-s*y, origin_world[1]+s*x+c*y
        if (abs(wx-center_world[0]) <= size_world[0]/2+1e-9
                and abs(wy-center_world[1]) <= size_world[1]/2+1e-9):
            values.append(int(value))
    return {'known_cells': len(values), 'maximum_cost': max(values) if values else None,
            'inscribed_or_obstacle_cells': sum(v >= 99 for v in values),
            'lethal_cells': sum(v == 100 for v in values)}
