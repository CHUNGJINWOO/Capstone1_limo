from pathlib import Path
import json,math,bisect
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import gi
gi.require_version('Gst','1.0');from gi.repository import Gst
Gst.init(None)
ROOT=Path(__file__).resolve().parents[4];DATA=ROOT/'output/simulation/2026-10-24';OUT=ROOT/'output/video';OUT.mkdir(exist_ok=True)
FPS=20;W=1920;H=1080
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc';BOLD='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
f42=ImageFont.truetype(BOLD,42);f30=ImageFont.truetype(BOLD,30);f26=ImageFont.truetype(FONT,26);f24=ImageFont.truetype(FONT,24)
navy=(22,50,79);teal=(8,127,140);gray=(94,111,125)
static=np.array(Image.open(Path(__file__).resolve().parent / 'limo_fortress_map.pgm'))<89
cases=[]
for name in ['baseline']:
 folder=DATA/name
 if not (folder/'summary.json').exists():continue
 records=[json.loads(x) for x in (folder/'telemetry.jsonl').read_text().splitlines()];summary=json.loads((folder/'summary.json').read_text());times=[r['sim_time'] for r in records];length=times[-1]-times[0]
 cases.append({'name':name,'folder':folder,'records':records,'summary':summary,'times':times,'length':length,'frames':math.ceil(length*FPS)})
assert cases
mp4=OUT/'Nav2_시뮬레이션_목표점주행.mp4';webm=OUT/'Nav2_시뮬레이션_목표점주행_Impress용.webm'
p=Gst.parse_launch(f'appsrc name=src is-live=false format=time block=true caps=video/x-raw,format=RGB,width={W},height={H},framerate={FPS}/1 ! queue ! videoconvert ! video/x-raw,format=I420 ! tee name=t t. ! queue ! x264enc speed-preset=veryfast tune=zerolatency bitrate=4500 key-int-max=40 ! video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! filesink location="{mp4}" t. ! queue ! vp8enc deadline=1 cpu-used=5 threads=4 target-bitrate=4500000 keyframe-max-dist=40 ! webmmux ! filesink location="{webm}"')
src=p.get_by_name('src');p.set_state(Gst.State.PLAYING);outidx=0;prevkey=None;cache=None
for caseidx,case in enumerate(cases):
 rs=case['records'];times=case['times'];folder=case['folder'];result=case['summary'];title='가상 실내 환경의 목표점 주행' if case['name']=='baseline' else '2. 지도에 없는 상자 추가'
 for frameidx in range(case['frames']):
  t=times[0]+frameidx/FPS;idx=max(0,bisect.bisect_right(times,t)-1);rec=rs[idx];key=(caseidx,idx)
  if key!=prevkey:
   im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
   d.rectangle((0,0,W,118),fill=navy);d.text((44,24),'시뮬레이션  ·  Nav2 목표점 주행',font=f42,fill='white');d.text((1270,42),'실제 LIMO 주행 결과와 구분',font=f26,fill=(178,221,227))
   d.text((44,138),title,font=f30,fill=teal);d.text((1060,138),'지도·현재 장애물·Nav2 경로',font=f30,fill=teal)
   camera=Image.open(folder/rec['image']).convert('RGB');camera=camera.crop((160,0,1120,960)).transpose(Image.ROTATE_270).resize((800,800),Image.LANCZOS);im.paste(camera,(44,192))
   # Render actual received global costmap and plan. This is topic visualization, not a fabricated RViz recording.
   cm=rec['costmap'];a=np.array(cm['data'],dtype=np.int16).reshape(cm['height'],cm['width']);a=np.flipud(a)
   rgb=np.zeros((a.shape[0],a.shape[1],3),dtype=np.uint8);rgb[:]=[243,247,248];rgb[a<0]=[197,204,210];rgb[(a>0)&(a<99)]=[188,224,230]
   occupied=a>=99;rgb[occupied]=[218,87,53]
   if static.shape==a.shape:rgb[static]=navy
   chart=Image.fromarray(rgb).resize((720,720),Image.NEAREST);cd=ImageDraw.Draw(chart)
   def point(x,y):return ((x-cm['origin'][0])/(cm['width']*cm['resolution'])*720,720-(y-cm['origin'][1])/(cm['height']*cm['resolution'])*720)
   plan=[point(x,y) for x,y in rec['plan']]
   if len(plan)>1:cd.line(plan,fill=(35,104,216),width=5)
   trail=[point(*r['pose'][:2]) for r in rs[:idx+1] if r['pose']]
   if len(trail)>1:cd.line(trail,fill=(145,110,20),width=3)
   gx,gy=point(1.5,0);cd.ellipse((gx-12,gy-12,gx+12,gy+12),outline=(26,153,59),width=4)
   if rec['pose']:
    x,y,ang=rec['pose'];px,py=point(x,y);cd.ellipse((px-10,py-10,px+10,py+10),fill=(26,153,59),outline='white',width=2);cd.line((px,py,px+28*math.cos(ang),py-28*math.sin(ang)),fill=(26,153,59),width=5)
   im.paste(chart,(1080,192));d=ImageDraw.Draw(im)
   d.text((1080,932),'벽: 남색   현재 관측: 주황   계획 경로: 파랑',font=f24,fill=gray)
   d.text((1080,972),'가상 로봇 이동 / 위치추정: 초록',font=f24,fill=gray)
   d.text((44,1007),rec['status'],font=f26,fill=teal)
   if case['name']=='obstacle':
    caption='저장 지도에는 상자 없음 / 센서 관측을 Costmap에 반영'
   else:caption='저장 지도에서 같은 목표점으로 이동'
   d.text((870,1007),caption,font=f24,fill=gray)
   cache=im;prevkey=key
  im=cache.copy();d=ImageDraw.Draw(im);d.rectangle((44,1048,1810,1054),fill=(220,231,235));d.rectangle((44,1048,44+int(1766*(frameidx+1)/case['frames']),1054),fill=teal);d.text((42,1057),f'시뮬레이션 시간 기준 1배속 / 구간 경과 {frameidx/FPS:04.1f}초',font=ImageFont.truetype(FONT,14),fill=gray)
  b=Gst.Buffer.new_allocate(None,W*H*3,None);b.fill(0,im.tobytes());b.pts=Gst.util_uint64_scale(outidx,Gst.SECOND,FPS);b.duration=Gst.util_uint64_scale(1,Gst.SECOND,FPS)
  assert src.emit('push-buffer',b)==Gst.FlowReturn.OK;outidx+=1
  if frameidx==int(case['frames']*.5):im.save(DATA/(case['name']+'_video_preview.png'))
 print('encoded section',case['name'],case['frames'],flush=True)
src.emit('end-of-stream');msg=p.get_bus().timed_pop_filtered(55*Gst.SECOND,Gst.MessageType.EOS|Gst.MessageType.ERROR)
if not msg:raise RuntimeError('encode timeout')
if msg.type==Gst.MessageType.ERROR:raise RuntimeError(msg.parse_error())
p.set_state(Gst.State.NULL)
(DATA/'video_metadata.json').write_text(json.dumps({'fps':FPS,'frames':outidx,'seconds':outidx/FPS,'time_basis':'simulation timestamps, 1x; missing camera frames held until next real observation','cases':[{'name':c['name'],'duration':c['length'],'action_status':c['summary']['action_status']} for c in cases],'source':'Gazebo camera and actual ROS topics, not physical LIMO; fixed box is not a moving obstacle'},indent=2,ensure_ascii=False))
print('DONE',outidx,mp4,webm,flush=True)
