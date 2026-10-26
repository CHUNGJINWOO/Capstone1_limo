"""Dense unmapped-obstacle experiment using a native simulation-step motion plugin."""
import copy,json,pathlib,sys,shutil,xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
HERE=pathlib.Path(__file__).resolve().parent
SIM=HERE.parent
sys.path.insert(0,str(SIM));sys.path.insert(0,str(SIM/'complex_indoor_env'))
from generate_complex_env import define_complex_geometry,build_occupancy_grid_and_verify,generate_stage_sdf

def prepare():
 target=HERE/'env';target.mkdir(exist_ok=True)
 original=SIM/'complex_indoor_env'
 manifest=json.loads((original/'complex_indoor_manifest.json').read_text())
 cfg=copy.deepcopy(manifest['stages']['E'])
 cfg.update(stage_id='J',name='dense_unmapped_continuous_loops',title='Stage J: 미등록 장애물 다수 + 연속 순환 이동 3개',timeout_sim=130.0)
 geom=define_complex_geometry()
 cfg['unmapped_obstacles'] += [{'name':n,'pose':[cx,cy,h/2],'size':[sx,sy,h],'role':'Removed from saved map; observed through scan only'} for n,cx,cy,sx,sy,h,_ in geom['mapped_static_obstacles']]
 cfg['unmapped_obstacles'] += [dict(name='extra_unmapped_1',pose=[-3.1,3.9,.35],size=[.5,.5,.7],role='unmapped'),dict(name='extra_unmapped_2',pose=[-3,-4.1,.35],size=[.5,.5,.7],role='unmapped')]
 loops=[(3.1,1.3,.35,1.4,24,0),(5.4,1.3,.35,1.7,28,2),(0.5,4.7,1.4,.35,26,1)]
 cfg['moving_obstacles']=[]
 for i,(cx,cy,rx,ry,period,phase) in enumerate(loops,1):
  import math
  pose=[cx+rx*math.cos(phase),cy+ry*math.sin(phase),.35]
  cfg['moving_obstacles'].append(dict(name=f'moving_obs_{i}',size=[.42,.42,.7],initial_pose=pose,start_pose=pose,end_pose=pose,speed_mps=0.0,trigger_type='always',trigger_value=0,clears_corridor=False,description='Native PreUpdate continuous ellipse; speed_mps=0 disables external pose controller',native_motion=dict(cx=cx,cy=cy,rx=rx,ry=ry,z=.35,period=period,phase=phase)))
 # Stored map contains structural walls only: all seven added fixed obstacles are unmapped.
 bare=copy.deepcopy(geom);bare['mapped_static_obstacles']=[]
 grid,_=build_occupancy_grid_and_verify(bare);Image.fromarray(grid).save(target/'map.pgm')
 (target/'map.yaml').write_text('image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-8.0, -6.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
 # Physical world keeps the former mapped pillars, now listed as unmapped.
 generate_stage_sdf(bare,cfg,target/'world.sdf')
 tree=ET.parse(target/'world.sdf');world=tree.getroot().find('world')
 for spec in cfg['moving_obstacles']:
  model=world.find(f"model[@name='{spec['name']}']")
  plugin=ET.SubElement(model,'plugin',filename=str(HERE/'build/liblimo_smooth_obstacle.so'),name='SmoothMotion')
  for k,v in spec['native_motion'].items():ET.SubElement(plugin,k).text=str(v)
 tree.write(target/'world.sdf',encoding='unicode',xml_declaration=True)
 for name in ['nav2_complex_indoor_ackermann_params.yaml','complex_indoor_nav2.rviz']:shutil.copy2(original/name,target/name)
 manifest.update(stages={'J':cfg},stage_worlds={'J':str(target/'world.sdf')},map_yaml=str(target/'map.yaml'),map_pgm=str(target/'map.pgm'))
 (target/'complex_indoor_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
 # Fixed obstacle centres must be free in the stored map.
 for spec in cfg['unmapped_obstacles']:
  x,y,_=spec['pose'];c=int((x+8)/.05);r=grid.shape[0]-1-int((y+6)/.05)
  assert grid[r,c]==254,(spec['name'],grid[r,c])
 print('Verified stored map: seven fixed obstacle centres are free; moving objects not written to map',flush=True)
 return target

if __name__=='__main__':
 target=prepare()
 if '--prepare-only' not in sys.argv:
  import interactive_sim_manager as manager
  manager.ENV_DIR=target;manager.RUNS_BASE_DIR=SIM/'2026-10-24_continuous_runs'
  manager.run_interactive_stage('J',184)
