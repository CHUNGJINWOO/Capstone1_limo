"""30fps explanatory visualization: interpolate recorded poses, never fabricate outcomes."""
import pathlib,json,bisect,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import gi
gi.require_version('Gst','1.0')
from gi.repository import Gst
Gst.init(None)
HERE=pathlib.Path(__file__).resolve().parent
run=sorted((HERE.parent/'2026-10-24_continuous_runs').glob('stage_J*'))[-1]
read=lambda n:[json.loads(l) for l in (run/n).open()]
gt=read('gt_pose_history.jsonl');obs=read('obstacle_pose_history.jsonl');plans=read('plan_history.jsonl')
summary=json.loads((run/'summary.json').read_text());spec=json.loads((run/'obstacle_scenario.json').read_text());rt=json.loads((run/'runtime_execution_record.json').read_text())
start=rt['phase_timestamps']['nav_start_sim'];end=rt['phase_timestamps']['nav_end_sim']
series={'robot':gt};series.update({s['name']:[r for r in obs if r['model_name']==s['name']] for s in spec['moving_obstacles']})
times={n:[r['recv_sim_time'] for r in a] for n,a in series.items()}
def pose(n,t):
 a=series[n];ts=times[n];i=bisect.bisect_right(ts,t)
 if i==0:return a[0]
 if i>=len(a):return a[-1]
 p,q=a[i-1],a[i];f=(t-ts[i-1])/max(1e-8,ts[i]-ts[i-1]);return {'x':p['x']+(q['x']-p['x'])*f,'y':p['y']+(q['y']-p['y'])*f,'yaw':p.get('yaw',0)+math.atan2(math.sin(q.get('yaw',0)-p.get('yaw',0)),math.cos(q.get('yaw',0)-p.get('yaw',0)))*f}
font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',19);big=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',24)
base=Image.new('RGB',(1280,720),'#132136');grid=Image.open(HERE/'env/map.pgm').convert('RGB').resize((768,576),Image.NEAREST);base.paste(grid,(28,100))
def xy(x,y):return (28+(x+8)*48,100+(6-y)*48)
def text(d,p,s,c='white',f=None):d.text(p,s,font=f or font,fill=c)
text(ImageDraw.Draw(base),(28,18),'시뮬레이션 · 미등록 고정 7개 + 연속 이동 3개',f=big)
text(ImageDraw.Draw(base),(28,55),'실행 로그 위치 보간 시각화 · 30 FPS · 1배속 (Gazebo 화면 녹화와 구분)')
colors=['#368dff','#aa55ee','#11b0ac'];fps=30;out=run/'continuous_obstacles_30fps.mp4'
pipe=Gst.parse_launch(f'appsrc name=src is-live=false format=time block=true caps=video/x-raw,format=RGB,width=1280,height=720,framerate=30/1 ! queue ! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=3500 ! video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{out}"');src=pipe.get_by_name('src');pipe.set_state(Gst.State.PLAYING)
plan_ts=[p['sim_time'] for p in plans]
for k,t in enumerate(np.arange(start,end,1/fps)):
 im=base.copy();d=ImageDraw.Draw(im)
 for s in spec['unmapped_static_obstacles']:
  x,y,_=s['pose'];w,h=s['size'][:2];a=xy(x-w/2,y+h/2);b=xy(x+w/2,y-h/2);d.rectangle([a,b],fill='#ee6262',outline='#a02222',width=2)
 if plans:
  p=plans[0]['poses'];pts=[xy(*q[:2]) for q in p]
  for i in range(0,len(pts)-2,6):d.line(pts[i:i+3],fill='#337cff',width=2)
  idx=max(0,bisect.bisect_right(plan_ts,t)-1);d.line([xy(*q[:2]) for q in plans[idx]['poses']],fill='#fda42b',width=3)
 trail=[xy(g['x'],g['y']) for g in gt if g['recv_sim_time']<=t]
 if len(trail)>1:d.line(trail,fill='#21b55e',width=3)
 for s,col in zip(spec['moving_obstacles'],colors):
  p=pose(s['name'],t);x,y=xy(p['x'],p['y']);d.rectangle([x-10,y-10,x+10,y+10],fill=col,outline='black',width=2)
 r=pose('robot',t);x,y=xy(r['x'],r['y']);a=r.get('yaw',0);d.ellipse([x-7,y-7,x+7,y+7],fill='#32ec71',outline='black',width=2);d.line([(x,y),(x+15*math.cos(a),y-15*math.sin(a))],fill='black',width=3)
 for label,p,col in [('S',summary['initial_pose'],'#337cff'),('G',summary['goal_pose'],'#e23232')]:
  x,y=xy(*p[:2]);d.ellipse([x-12,y-12,x+12,y+12],fill=col);text(d,(x-6,y-14),label)
 text(d,(830,115),f'주행 경과: {t-start:05.1f}초',f=big)
 for yy,st in [(170,'빨강: 저장 지도에 없는 고정 장애물'),(210,'청색·보라·청록: 연속 순환 이동'),(270,'파란 점선: 최초 경로'),(310,'주황 실선: 현재 계획 경로'),(350,'초록 실선: 실제 주행 궤적'),(410,'장애물 위치는 평가용 표시이며'),(438,'Nav2에는 센서 관측만 제공'),(510,'결과: 목표 도달 / 안전 기준 미통과'),(550,'샘플링 2D 형상 겹침 기록 있음'),(590,'자동차 물리 모델이 아닌 지정 궤적'),(618,'이동체에 대한 대응 시험')]:text(d,(822,yy),st,'#ffaaaa' if yy in (510,550) else 'white')
 buf=Gst.Buffer.new_allocate(None,len(im.tobytes()),None);buf.fill(0,im.tobytes());buf.pts=Gst.util_uint64_scale(k,Gst.SECOND,fps);buf.duration=Gst.util_uint64_scale(1,Gst.SECOND,fps);src.emit('push-buffer',buf)
 if k==int((end-start)*fps/2):im.save(run/'continuous_representative.png')
src.emit('end-of-stream');msg=pipe.get_bus().timed_pop_filtered(60*Gst.SECOND,Gst.MessageType.EOS|Gst.MessageType.ERROR)
if msg is None or msg.type==Gst.MessageType.ERROR:raise RuntimeError('video encoding failed')
pipe.set_state(Gst.State.NULL);print(out)
