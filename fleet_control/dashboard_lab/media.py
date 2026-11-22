"""One capture/encode pipeline, independent RGB/depth rates, bounded latest-frame fanout."""
import asyncio, collections, dataclasses, hashlib, io, json, math, random, struct, time, zlib
from fractions import Fraction
from pathlib import Path
import av
import cv2
import numpy as np

PROFILES = {'high': (640, 75, 15), 'balanced': (480, 65, 12), 'low': (320, 45, 8), 'emergency': (320, 35, 5)}
FAULTS = {
 'normal': dict(rtt_ms=0,jitter_ms=0,loss=0,kbps=0,offline_s=0),
 'rtt400': dict(rtt_ms=400,jitter_ms=0,loss=0,kbps=0,offline_s=0),
 'jitter': dict(rtt_ms=60,jitter_ms=120,loss=0,kbps=0,offline_s=0),
 'loss10': dict(rtt_ms=60,jitter_ms=20,loss=.1,kbps=0,offline_s=0),
 'bandwidth150': dict(rtt_ms=20,jitter_ms=0,loss=0,kbps=150,offline_s=0),
 'disconnect': dict(rtt_ms=0,jitter_ms=0,loss=0,kbps=0,offline_s=3),
 'stalequeue': dict(rtt_ms=700,jitter_ms=120,loss=.15,kbps=250,offline_s=0),
}

def percentiles(values):
 a=np.asarray(list(values),float)
 return {'n':len(a),'mean':round(float(a.mean()),3),'p95':round(float(np.percentile(a,95)),3),'p99':round(float(np.percentile(a,99)),3)} if a.size else {'n':0,'mean':None,'p95':None,'p99':None}

def key(h):return (h['boot'],h['stream'],h['sequence'],h['frame_id'],h['stamp_ns'])
def envelope(header,payload):
 raw=json.dumps(header,separators=(',',':'),allow_nan=False).encode()
 return struct.pack('!I',len(raw))+raw+payload

def decode_envelope(data):
 n=struct.unpack('!I',data[:4])[0]
 if n>65536 or len(data)<4+n:raise ValueError('frame envelope')
 return json.loads(data[4:4+n]),data[4+n:]

class Latest:
 def __init__(self):self.value=None;self.version=0;self.condition=asyncio.Condition();self.overwritten=0
 async def put(self,value):
  async with self.condition:self.value=value;self.version+=1;self.condition.notify_all()
 async def get(self,after):
  async with self.condition:
   await self.condition.wait_for(lambda:self.version>after)
   return self.version,self.value

@dataclasses.dataclass
class Packet:
 header:dict
 payload:bytes
 rgb:object=None

class Hub:
 def __init__(self,assets,recorded=None,mode='SIMULATION',inference=None):
  import uuid
  self.boot=uuid.uuid4().hex;self.mode=mode;self.assets=assets;self.recorded=recorded;self.inference=inference or {}
  self.rgb=Latest();self.depth=Latest();self.h264=Latest();self.raw=Latest();self.metrics=collections.Counter();self.timings=collections.defaultdict(lambda:collections.deque(maxlen=4000));self.profile='balanced';self.adaptive=True;self.fault='normal';self.fault_started=time.monotonic();self.raw_depth=collections.OrderedDict();self.stop=False
  self.encoded_per_frame=collections.Counter();self.measurement_stage='manual';self.fault_history=collections.deque(maxlen=200);self.fault_history.append({'t':time.time(),'fault':'normal','stage':'initial'});self.profile_history=collections.deque(maxlen=100);self.profile_history.append({'t':time.time(),'profile':self.profile,'reason':'initial'})
  self.delivery_ema_ms=None;self.sequence=0;self.origin=time.time();self.rng=random.Random(461);self.tasks=[];self.h264_enabled=False;self.codec=None;self.last_fault_change=0
  self.replay_depth=None
  if recorded and recorded.get('depth'):
   d=recorded['depth'];self.replay_depth=np.frombuffer(zlib.decompress(__import__('base64').b64decode(d['values_f32le_zlib'])),dtype='<f4').reshape(d['height'],d['width']).copy()
 def set_fault(self,name):
  if name not in FAULTS:raise ValueError('fault name')
  self.fault_history.append({'t':time.time(),'fault':name,'stage':self.measurement_stage});self.delivery_ema_ms=None;self.fault=name;self.fault_started=time.monotonic();self.last_fault_change=time.time();self.choose_profile()
 def choose_profile(self):
  if not self.adaptive:return
  f=FAULTS[self.fault];name='emergency' if f['kbps'] and f['kbps']<300 else 'low' if f['rtt_ms']>=250 or f['loss'] else 'balanced'
  if name!=self.profile:self.profile=name;self.profile_history.append({'t':time.time(),'profile':name,'reason':self.fault})
 def observe_delivery(self,milliseconds):
  if not math.isfinite(milliseconds) or milliseconds<0:return
  self.delivery_ema_ms=milliseconds if self.delivery_ema_ms is None else .8*self.delivery_ema_ms+.2*milliseconds
  self.timings['delivery_service_ms'].append(milliseconds)
  if not self.adaptive:return
  target='emergency' if self.delivery_ema_ms>350 else 'low' if self.delivery_ema_ms>130 else 'balanced' if self.delivery_ema_ms<70 else self.profile
  if target!=self.profile:self.profile=target;self.profile_history.append({'t':time.time(),'profile':target,'reason':'measured delivery EMA'})
 def config(self):
  w,q,fps=PROFILES[self.profile];return {'rgb_width':w,'jpeg_quality':q,'rgb_fps_cap':fps,'depth_width':320,'depth_fps_cap':8,'depth_quality':60,'adaptive':self.adaptive,'profile':self.profile,'fault':self.fault}
 async def start(self):
  self.tasks=[asyncio.create_task(fn()) for fn in [self.capture,self.encode_rgb,self.encode_depth,self.encode_h264]]
 async def close(self):
  self.stop=True
  for t in self.tasks:t.cancel()
  await asyncio.gather(*self.tasks,return_exceptions=True)
 async def capture(self):
  deadline=time.monotonic()
  while not self.stop:
   t=time.perf_counter();self.sequence+=1;seq=self.sequence
   original=self.assets[(seq//30)%len(self.assets)]
   rgb=original['rgb'].copy();stamp=time.time_ns()
   # Synthetic measurement overlay on a COPY, never overwrite saved source images.
   bits=f'{seq & 0xffffffff:032b}{(stamp//1000000)&0xffffffff:032b}'
   for i,b in enumerate(bits):rgb[0:16,int(i*rgb.shape[1]/64):int((i+1)*rgb.shape[1]/64)]=255 if b=='1' else 0
   h={'boot':self.boot,'sequence':seq,'frame_id':original['id'],'stamp_ns':str(stamp),'capture_wall_ms':stamp/1e6,'source':'RECORDED_GAZEBO_RGB','mode':'REPLAY' if self.mode=='REPLAY' else 'SIMULATION','original_sha256':original['sha256'],'measurement_overlay':True}
   await self.raw.put((h,rgb));self.timings['capture_ms'].append((time.perf_counter()-t)*1000);self.metrics['captured']+=1
   deadline+=1/30;await asyncio.sleep(max(0,deadline-time.monotonic()))
 async def encode_rgb(self):
  seen=0
  while not self.stop:
   started=time.monotonic();version,(h,rgb)=await self.raw.get(seen);self.metrics['rgb_source_skipped']+=max(0,version-seen-1);seen=version
   w,q,fps=PROFILES[self.profile];t=time.perf_counter();image=cv2.resize(rgb,(w,round(w*rgb.shape[0]/rgb.shape[1])))
   ok,jpg=await asyncio.to_thread(cv2.imencode,'.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,q]);encode_ms=(time.perf_counter()-t)*1000
   if not ok:continue
   inference=self.inference.get(h['original_sha256']);detections=[]
   if inference:
    sx=w/rgb.shape[1];sy=image.shape[0]/rgb.shape[0]
    detections=[{**d,'x1':d['x1']*sx,'x2':d['x2']*sx,'y1':d['y1']*sy,'y2':d['y2']*sy} for d in inference['detections']]
   header={**h,'stream':'rgb','codec':'jpeg','width':w,'height':image.shape[0],'encode_ms':encode_ms,'encoded_wall_ms':time.time()*1000,
    'detections':detections,'result_key':list(key(dict(h,stream='rgb'))),'inference_mode':'PRECOMPUTED_CPU_REPLAY' if inference else 'UNAVAILABLE','inference_ms_original':inference.get('infer_ms') if inference else None,'profile':self.profile}
   await self.rgb.put(Packet(header,jpg.tobytes(),image));self.metrics['rgb_encoded']+=1;self.metrics['rgb_encoded_bytes']+=len(jpg);self.timings['jpeg_encode_ms'].append(encode_ms)
   await asyncio.sleep(max(0,1/fps-(time.monotonic()-started)))
 async def encode_depth(self):
  y,x=np.mgrid[0:200,0:320];base=(.35+x/100+y/400).astype('<f4');base[(x-70)**2+(y-100)**2<26**2]=np.nan
  while not self.stop:
   started=time.monotonic();self.metrics['depth_captured']+=1;seq=self.metrics['depth_captured'];stamp=time.time_ns()
   replay=self.mode=='REPLAY' and self.replay_depth is not None
   values=self.replay_depth.copy() if replay else (base+.1*math.sin(seq/10)).astype('<f4')
   if replay:encoding='32FC1';raw=values.astype('<f4').tobytes();scale=1.;unit_verified=False
   else:
    mm=np.nan_to_num(values*1000,nan=0).astype('<u2');values=mm.astype('<f4')*.001;values[mm==0]=np.nan;raw=mm.tobytes();encoding='16UC1';scale=.001;unit_verified=True
   t=time.perf_counter();valid=np.isfinite(values)&(values>0);normalized=np.nan_to_num(np.clip((values-.2)/3.8,0,1),nan=0)
   color=cv2.applyColorMap((normalized*255).astype(np.uint8),cv2.COLORMAP_TURBO);color[~valid]=[40,45,53]
   ok,jpg=await asyncio.to_thread(cv2.imencode,'.jpg',color,[cv2.IMWRITE_JPEG_QUALITY,60]);duration=(time.perf_counter()-t)*1000
   identifier=f'{self.boot}:depth:{seq}'
   h={'boot':self.boot,'sequence':seq,'stream':'depth','frame_id':'recorded_depth_optical' if replay else 'SYNTHETIC_depth_optical','stamp_ns':str(stamp),'capture_wall_ms':stamp/1e6,'encoded_wall_ms':time.time()*1000,'width':values.shape[1],'height':values.shape[0],'codec':'jpeg','depth_key':identifier,'encoding':encoding,'scale_m':scale,'unit':'m','unit_verified':unit_verified,'source':'SAVED_REAL_METRIC_SAMPLE' if replay else 'SYNTHETIC_DEPTH','source_stamp_ns':str(self.recorded['depth']['stamp_ns']) if replay else str(stamp),'rgb_registered':False,'encode_ms':duration,'valid_fraction':float(valid.mean())}
   self.raw_depth[identifier]=(dict(h,raw_dtype='<f4' if replay else '<u2',distance_payload='lossless stored metric sample' if replay else 'synthetic native 16UC1'),zlib.compress(raw,1))
   while len(self.raw_depth)>32:self.raw_depth.popitem(last=False)
   if ok:await self.depth.put(Packet(h,jpg.tobytes()));self.metrics['depth_encoded']+=1;self.metrics['depth_bytes']+=len(jpg)
   self.timings['depth_color_encode_ms'].append(duration);await asyncio.sleep(max(0,1/8-(time.monotonic()-started)))
 async def encode_h264(self):
  seen=0
  while not self.stop:
   version,packet=await self.rgb.get(seen);seen=version
   if not self.h264_enabled:continue
   image=packet.rgb;h,w=image.shape[:2]
   if self.codec is None or (self.codec.width,self.codec.height)!=(w,h):
    self.codec=av.CodecContext.create('libx264','w');self.codec.width=w;self.codec.height=h;self.codec.pix_fmt='yuv420p';self.codec.time_base=Fraction(1,1000000);self.codec.framerate=Fraction(15,1);self.codec.bit_rate=600000;self.codec.options={'preset':'ultrafast','tune':'zerolatency','profile':'baseline','g':'15','bf':'0'}
   t=time.perf_counter();frame=av.VideoFrame.from_ndarray(image,format='bgr24');frame.pts=int(packet.header['capture_wall_ms']*1000);frame.time_base=Fraction(1,1000000)
   chunks=await asyncio.to_thread(self.codec.encode,frame)
   for chunk in chunks:
    header={**packet.header,'codec':'h264','codec_string':'avc1.42E01E','keyframe':chunk.is_keyframe,'h264_encode_ms':(time.perf_counter()-t)*1000,'timestamp_us':frame.pts}
    await self.h264.put(Packet(header,bytes(chunk)));self.metrics['h264_encoded']+=1;self.metrics['h264_bytes']+=chunk.size
   self.timings['h264_encode_ms'].append((time.perf_counter()-t)*1000)
 async def impairment(self,size,transport='tcp'):
  f=FAULTS[self.fault]
  if f['offline_s'] and time.monotonic()-self.fault_started<f['offline_s']:self.metrics['offline_dropped']+=1;await asyncio.sleep(.05);return False
  delay=f['rtt_ms']/2000+self.rng.uniform(0,f['jitter_ms']/1000)
  # App-level link model, NOT a kernel packet-loss claim: TCP retransmission/HOL stall.
  if transport!='udp' and self.rng.random()<f['loss']:
   self.metrics['simulated_lost_units']+=1
   if transport=='udp':return False
   delay+=.18
  if f['kbps']:delay+=size*8/(f['kbps']*1000)
  await asyncio.sleep(delay)
  return True
 def snapshot(self):return {'config':self.config(),'counters':dict(self.metrics),'stages':{k:percentiles(v) for k,v in self.timings.items()},'profile_history':list(self.profile_history),'fault_history':list(self.fault_history),'boot':self.boot,'source_mode':self.mode,'delivery_ema_ms':self.delivery_ema_ms,'task_errors':[type(t.exception()).__name__+': '+str(t.exception()) for t in self.tasks if t.done() and not t.cancelled() and t.exception() is not None]}
