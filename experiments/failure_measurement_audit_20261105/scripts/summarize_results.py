#!/usr/bin/env python3
"""원시 결과를 보존하면서 계측 범위와 관측 결과를 별도 파일로 요약한다."""
import json
from pathlib import Path

EXP = Path(__file__).resolve().parents[1]
ROOT = EXP.parents[1]
OUT = ROOT / 'notebook/2026_11_05/06_실패시험_계측교정'


def summarize(directory):
    result = json.loads((directory / 'result.json').read_text())
    world = []
    create = remove = None
    center_values = []
    with (directory / 'telemetry.jsonl').open() as stream:
        for line in stream:
            item = json.loads(line)
            if item['kind'] == 'gazebo_world_pose':
                world.append(item)
            elif item['kind'] == 'center_cost' and item['value'] is not None:
                center_values.append(item['value'])
            elif item['kind'] == 'dynamic_box_create_response' and item['created']:
                create = item['wall_monotonic']
            elif item['kind'] == 'dynamic_box_remove_response' and item['success']:
                remove = item['wall_monotonic']
    events = result.get('explorer_events', [])
    trajectory = result.get('world_trajectory', {})
    terminal = result.get('action_result') or {}
    observation = {
        'run_id': result['run_id'], 'scenario': result['scenario'],
        'error': result['error'], 'condition_created': result['condition_created'],
        'contact_pipeline': result.get('contact_status'),
        'obstacle_contact_samples': result.get('obstacle_contact_samples'),
        'contact_pair_episodes': len(result.get('obstacle_contact_episodes', [])),
        'world_unique_samples': trajectory.get('samples'),
        'displacement_m': trajectory.get('displacement_m'),
        'cumulative_distance_m': trajectory.get('cumulative_distance_m'),
        'world_xy_bounds': {key: [min(p[key] for p in world), max(p[key] for p in world)]
                            for key in ('x', 'y')} if world else None,
        'initial_center_cost': result.get('initial_center_cost'),
        'center_cost_range': [min(center_values), max(center_values)] if center_values else None,
        'goal_dispatches': sum(e['event'] == 'goal_sent' for e in events),
        'failure_counts': [e['fails'] for e in events if e['event'] == 'goal_excluded'],
        'retry_limit_reached': any(e.get('retry_limit_reached', False) for e in events),
        'terminal_status': terminal.get('status'), 'terminal_message': terminal.get('message'),
        'saved_files': result['saved_files'], 'tester_cancel': result.get('tester_cancel'),
        'wheel_slip': 'not_measured',
        'dynamic_obstacle': result['dynamic_obstacle'],
    }
    if create is not None:
        # 배치·제거와 이동의 시간 관계만 기록한다. 정지/재출발 성공 판정으로 바꾸지 않는다.
        from audit_metrics import trajectory_metrics
        observation['dynamic_motion_phases'] = {}
        for name, lower, upper in (('before_creation', float('-inf'), create),
                                   ('while_present', create, remove or float('inf')),
                                   ('after_removal', remove or float('inf'), float('inf'))):
            points = [p for p in world if lower <= p['wall_monotonic'] < upper]
            observation['dynamic_motion_phases'][name] = trajectory_metrics(points)
    return observation


if __name__ == '__main__':
    rows = [summarize(p.parent) for p in sorted((OUT / 'runs').glob('*/result.json'))]
    (OUT / 'records/observations.json').write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + '\n')
    print(f'{len(rows)} runs summarized; original result.json files preserved')
