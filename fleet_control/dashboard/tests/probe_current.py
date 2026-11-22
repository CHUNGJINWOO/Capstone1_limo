"""Inspect only the isolated visualization already started by run_local.py."""
import argparse,json,time,urllib.request
from pathlib import Path
import psutil
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=10);args=p.parse_args()
state=Path.home()/'.local/state/capstone-dashboard-visualization';token=(state/'viewer.token').read_text().strip()
def snapshot(port):
 r=urllib.request.Request(f'http://127.0.0.1:{port}/api/state',headers={'Authorization':'Bearer '+token})
 return json.load(urllib.request.urlopen(r,timeout=3))
registry=json.loads((state/'processes.json').read_text());processes={}
for name,item in registry.items():
 try:
  process=psutil.Process(item['pid']);ticks=Path(f'/proc/{process.pid}/stat').read_text().rpartition(') ')[2].split()[19]
  if ticks==item['start_ticks']:processes[name]=(process,sum(process.cpu_times()[:2]))
 except (OSError,psutil.Error):pass
first={p:snapshot(p) for p in [18210,18211]};start=time.monotonic();time.sleep(args.seconds);elapsed=time.monotonic()-start
last={p:snapshot(p) for p in first};report={'duration_s':round(elapsed,2),'processes':{},'gateways':{}}
for name,(process,cpu) in processes.items():
 try:report['processes'][name]={'cpu_percent_one_core':round((sum(process.cpu_times()[:2])-cpu)*100/elapsed,2),'rss_mib':round(process.memory_info().rss/2**20,1)}
 except psutil.Error:pass
for port,data in last.items():
 m=data['gateway_metrics'];old=first[port]['gateway_metrics']
 report['gateways'][port]={'upstream_reported_kB_s':round((m['upstream_bytes']-old['upstream_bytes'])/elapsed/1000,2),'browser_aggregate_kB_s':round((m['browser_bytes']-old['browser_bytes'])/elapsed/1000,2),'mm_polls_s':round((m['mm_requests']-old['mm_requests'])/elapsed,2),'video_polls_s':round((m['video_requests']-old['video_requests'])/elapsed,2),'pair_mismatch':m['pair_mismatch'],'bridge':data['visualization']['counters'],'layer_status':{k:v['status'] for k,v in data['visualization']['layers'].items()},'depth_status':(data['visualization'].get('depth')or{}).get('status'),'video_fresh':data['video']['fresh']}
v=last[18211]['visualization'];assert all(v['layers'][k]['status']=='OK' for k in ['map','global_costmap','local_costmap']);assert v['depth']['status']=='OK';assert last[18211]['video']['fresh'];assert v['counters']['rejected']==0
print(json.dumps(report,indent=2))
