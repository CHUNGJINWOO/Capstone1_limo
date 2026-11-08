"""Additional deterministic trials; reuses the existing manager without changing navigation."""
import argparse
import copy
import json
import pathlib
import shutil
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent

def prepare():
    original = ROOT / 'complex_indoor_env'
    target = ROOT / 'oscillating_env'
    target.mkdir(exist_ok=True)
    manifest = json.loads((original / 'complex_indoor_manifest.json').read_text())
    base = manifest['stages']['E']
    for name in ['nav2_complex_indoor_ackermann_params.yaml', 'complex_indoor_nav2.rviz']:
        shutil.copy2(original / name, target / name)
    stages, worlds = {}, {}
    for key in ['G', 'H']:
        cfg = copy.deepcopy(base)
        cfg.update(stage_id=key, timeout_sim=110.0)
        mo = copy.deepcopy(base['moving_obstacles'][0])
        if key == 'G':
            p0, p1 = [2.9, 2.65, .35], [2.9, -.35, .35]
            cfg.update(name='fixed_plus_vertical_oscillation', title='Stage G: 고정 장애물 우회 + 상하 반복 왕복 장애물', description='Fixed unmapped obstacle and persistent vertical back-and-forth motion')
            mo.update(speed_mps=.4, trigger_value=.8)
        else:
            p0, p1 = [6.7, 2.25, .35], [3.0, 2.25, .35]
            cfg.update(name='fixed_plus_headon_oscillation', title='Stage H: 고정 장애물 우회 + 정면 접근 왕복 장애물', description='Observe stopping/avoidance without forced robot commands', goal_pose=[6.2,2.25,0.0])
            mo.update(speed_mps=.22, trigger_value=2.7)
        mo.update(initial_pose=p0, start_pose=p0, end_pose=p1, trigger_type='robot_x_ge', clears_corridor=False,
                  waypoints=[list(p) for _ in range(20) for p in (p0,p1)] + [p0],
                  description='20 deterministic round trips; no future trajectory supplied to Nav2')
        cfg['moving_obstacles']=[mo]
        cfg['expected_behavior']='Evaluate detection, path change, actual standstill, avoidance and arrival separately'
        tree=ET.parse(manifest['stage_worlds']['E']);world=tree.getroot().find('world')
        for model in list(world.findall('model')):
            if model.get('name') == 'moving_obs_2': world.remove(model)
            elif model.get('name') == 'moving_obs_1': model.find('pose').text=' '.join(map(str,p0+[0,0,0]))
        out=target/f'stage_{key}.sdf';tree.write(out,encoding='unicode',xml_declaration=True)
        stages[key]=cfg;worlds[key]=str(out)
    manifest['stages']=stages;manifest['stage_worlds']=worlds
    (target/'complex_indoor_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return target

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=['G','H']);ap.add_argument('--domain-id',type=int,default=182);ap.add_argument('--prepare-only',action='store_true');args=ap.parse_args()
    target=prepare()
    if not args.prepare_only:
        if args.stage is None: ap.error('--stage required')
        import interactive_sim_manager as manager
        manager.ENV_DIR=target
        manager.RUNS_BASE_DIR=ROOT/'2026-10-24_oscillating_runs'
        manager.run_interactive_stage(args.stage,args.domain_id)
