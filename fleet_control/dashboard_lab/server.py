"""Laptop-only isolated server. Fixed local assets; no SSH, ROS, robot commands, or remote upstream."""
import argparse, asyncio, base64, collections, hashlib, hmac, json, os, secrets, struct, time, uuid
from fractions import Fraction
from pathlib import Path
import av,cv2,numpy as np
from aiohttp import web,WSMsgType
from aiortc import MediaStreamTrack,RTCPeerConnection,RTCSessionDescription,RTCRtpSender,RTCConfiguration
from dashboard.server import wire_numbers
from .media import Hub,Packet,FAULTS,envelope,key,percentiles
from .state import ReadModel
ROOT=Path(__file__).parent

class SharedH264(MediaStreamTrack):
 kind='video'
 def __init__(self,lab):super().__init__();self.lab=lab;self.seen=0;self.ready=False
 async def recv(self):
  while True:
   self.seen,p=await self.lab.hub.h264.get(self.seen)
   if not self.ready and not p.header['keyframe']:continue
   self.ready=True
   if not await self.lab.hub.impairment(len(p.payload),'udp'):continue
   if time.time()*1000-p.header['capture_wall_ms']>700:self.lab.hub.metrics['webrtc_stale_dropped']+=1;continue
   packet=av.Packet(p.payload);packet.pts=p.header['timestamp_us'];packet.dts=packet.pts;packet.time_base=Fraction(1,1000000);return packet

class Lab:
 def __init__(self,args):
  self.args=args;self.state=Path(args.state).expanduser();self.state.mkdir(parents=True,exist_ok=True);self.state.chmod(0o700)
  self.viewer=self.token('viewer.token');self.tester=self.token('lab.token');self.sessions={};self.session_file=self.state/'sessions.json'
  if self.session_file.exists():self.sessions=json.loads(self.session_file.read_text())
  assets=[]
  for p in sorted(Path(args.assets).expanduser().glob('*.jpg')):
   b=p.read_bytes();im=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_COLOR)
   if im is not None:assets.append({'id':p.name,'sha256':hashlib.sha256(b).hexdigest(),'rgb':im})
  if not assets:raise ValueError('Saved JPEG assets required; no camera opened')
  recorded=json.loads(Path(args.recorded).expanduser().read_text()) if args.recorded else None
  inference=json.loads(Path(args.inference).read_text()) if args.inference else None
  self.hub=Hub(assets,recorded,args.mode,inference);self.model=ReadModel(self.hub,recorded);self.current={};self.peers=set();self.connections=set();self.telemetry=collections.defaultdict(lambda:collections.deque(maxlen=20000));self.counters=collections.Counter();self.frame_meta=collections.OrderedDict();self.since=time.time();self.tasks=[];self.tcp=None;self.client_seen={};self.failures=collections.deque(maxlen=100)
 def token(self,name):
  p=self.state/name
  if not p.exists():p.write_text(secrets.token_urlsafe(32)+'\n');p.chmod(0o600)
  if p.stat().st_mode&0o077:raise ValueError('credential permissions')
  return p.read_text().strip()
 def save_sessions(self):
  self.session_file.write_text(json.dumps(self.sessions));self.session_file.chmod(0o600)
 def role(self,r):
  a=r.headers.get('Authorization','')
  for name,t in [('viewer',self.viewer),('lab',self.tester)]:
   if a.startswith('Bearer ') and hmac.compare_digest(a[7:],t):return name
  session=self.sessions.get(r.cookies.get('capstone_lab',''))
  return session['role'] if session and session['expires']>time.time() else None
 def origin(self,r):return not r.headers.get('Origin') or r.headers['Origin']=='http://'+r.host
 async def startup(self,app):
  await self.hub.start();self.tasks=[asyncio.create_task(self.update()),asyncio.create_task(self.cache_meta())]
  self.tcp=await asyncio.start_server(self.tcp_client,'127.0.0.1',18313)
 async def shutdown(self,app):
  await asyncio.gather(*(ws.close(code=1001,message=b'lab restart') for ws in list(self.connections)),return_exceptions=True)
  await asyncio.gather(*(p.close() for p in list(self.peers)),return_exceptions=True)
 async def cleanup(self,app):
  await self.hub.close()
  for t in self.tasks:t.cancel()
  await asyncio.gather(*self.tasks,return_exceptions=True)
  await asyncio.gather(*(p.close() for p in self.peers),return_exceptions=True)
  self.tcp.close();await self.tcp.wait_closed()
 async def update(self):
  while True:
   self.current=self.model.update();await self.external_state();self.current['media']=self.hub.snapshot();self.current['clock_ms']=time.time()*1000
   # Legacy baseline receives the same sampled sensor source at its original 1Hz bridge cap.
   if self.args.baseline_url and self.hub.mode!='LIVE':
    try:
     import aiohttp,copy
     p=copy.deepcopy(self.current['fixture_packet']);p['sequence']=int(time.time()*10)
     d=self.hub.depth.value
     if d:
      meta,raw=self.hub.raw_depth[d.header['depth_key']];values=np.frombuffer(__import__('zlib').decompress(raw),dtype=meta['raw_dtype']).astype('<f4')*meta['scale_m'];values[values<=0]=np.nan
      from dashboard.core import pack
      p['depth']={**d.header,'values_f32le_zlib':pack(values.tobytes()),'width':meta['width'],'height':meta['height'],'source_width':meta['width'],'source_height':meta['height'],'scale_m':meta['scale_m'],'source_age_s':0}
     async with aiohttp.ClientSession() as s:
      async with s.post(self.args.baseline_url+'/api/bridge',json=p,headers={'Authorization':'Bearer '+self.args.baseline_token},timeout=aiohttp.ClientTimeout(total=.4)) as response:await response.read()
    except Exception:self.counters['baseline_unavailable']+=1
   await asyncio.sleep(1)
 async def external_state(self):
  if self.hub.mode!='LIVE':return
  if self.args.mission_url:
   try:
    import aiohttp
    token=Path(self.args.mission_token_file).expanduser().read_text().strip()
    async with aiohttp.ClientSession() as session:
     async with session.get(self.args.mission_url+'/api/v1/dashboard',headers={'Authorization':'Bearer '+token},timeout=aiohttp.ClientTimeout(total=.5)) as response:
      if response.status!=200:raise ValueError('mission read failed')
      data=await response.json()
     for route,name in [('/api/v1/missions?limit=500','missions'),('/api/v1/events?limit=500','events')]:
      async with session.get(self.args.mission_url+route,headers={'Authorization':'Bearer '+token},timeout=aiohttp.ClientTimeout(total=.5)) as detail:
       if detail.status==200:data[name]=(await detail.json()).get(name,[])
     for event in data.get('events',[]):event['severity']={'WARN':'WARNING','WARNING':'WARNING','ERROR':'ERROR','INFO':'INFO'}.get(str(event.get('level','')).upper(),'UNCLASSIFIED')
     for mission in data.get('missions',[]):
      mission['source']='LIVE_READ_API';mission['history']=[{'at':event['ts'],'status':str(event['kind'])+': '+str(event['message'])} for event in reversed(data.get('events',[])) if event.get('mission_id')==mission['mission_id']]
     self.current['mission']={**data,'connected':True,'source':'LIVE_READ_API'}
   except Exception:self.current['mission']['connected']=False
  if self.args.adapter_snapshot:
   try:
    data=json.loads(Path(self.args.adapter_snapshot).expanduser().read_text());age=time.time()-float(data['observed_at']);assert age>=-.2
    self.current['rmf']={'status':'CONNECTED' if age<=5 else 'STALE','registered':data.get('registered') if age<=5 else None,'fleet':str(data.get('fleet','')),'robots':list(data.get('robots',[])),'last_seen_s':age,'source':'CONFIGURED_READONLY_SNAPSHOT'}
   except Exception:self.current['rmf']={'status':'UNAVAILABLE','registered':None}
 async def cache_meta(self):
  seen=0
  while True:
   seen,p=await self.hub.rgb.get(seen);self.frame_meta[p.header['sequence']]=p.header
   while len(self.frame_meta)>128:self.frame_meta.popitem(last=False)
 async def tcp_client(self,reader,writer):
  try:
   hello=json.loads(await asyncio.wait_for(reader.readline(),3));assert hmac.compare_digest(str(hello.get('token','')),self.viewer)
   stream=self.hub.h264 if hello.get('codec')=='h264' else self.hub.rgb
   if stream is self.hub.h264:self.hub.h264_enabled=True
   seen=0
   while True:
    seen,p=await stream.get(seen)
    if not await self.hub.impairment(len(p.payload)):continue
    if time.time()*1000-p.header['capture_wall_ms']>700:self.hub.metrics['tcp_stale_dropped']+=1;continue
    body=envelope(p.header,p.payload);writer.write(struct.pack('!I',len(body))+body);await asyncio.wait_for(writer.drain(),.5);self.counters['tcp_bytes']+=len(body)
  except (Exception,asyncio.CancelledError):pass
  finally:writer.close();await writer.wait_closed()
 async def api(self,r):
  path=r.path
  if path=='/health':return web.json_response({'read_only':True,'boot':self.hub.boot})
  if path=='/api/login' and r.method=='POST':
   if not self.origin(r):raise web.HTTPForbidden()
   if sum(x>time.time()-60 for x in self.failures)>10:raise web.HTTPTooManyRequests()
   try:t=(await r.json())['token']
   except Exception:raise web.HTTPBadRequest()
   role='lab' if isinstance(t,str) and hmac.compare_digest(t,self.tester) else 'viewer' if isinstance(t,str) and hmac.compare_digest(t,self.viewer) else None
   if not role:self.failures.append(time.time());raise web.HTTPUnauthorized()
   sid=secrets.token_urlsafe(32);self.sessions[sid]={'role':role,'expires':time.time()+8*3600};self.save_sessions();res=web.json_response({'role':role});res.set_cookie('capstone_lab',sid,httponly=True,samesite='Strict',max_age=8*3600);return res
  role=self.role(r)
  if not role:raise web.HTTPUnauthorized()
  if r.method=='POST' and not self.origin(r):raise web.HTTPForbidden()
  if path=='/api/logout' and r.method=='POST':
   self.sessions.pop(r.cookies.get('capstone_lab',''),None);self.save_sessions();res=web.json_response({'ok':True});res.del_cookie('capstone_lab');return res
  if path=='/api/lab/control' and r.method=='POST':
   if role!='lab':raise web.HTTPForbidden()
   p=await r.json()
   if p.get('mode')=='REPLAY' and self.model.recorded is None:raise web.HTTPConflict(text='Replay dataset unavailable')
   if p.get('mode') in ['LIVE','REPLAY','SIMULATION']:self.hub.mode=p['mode']
   if self.hub.mode=='LIVE' and any(k in p for k in ['fault','profile']):raise web.HTTPForbidden(text='No fixture faults in LIVE')
   if p.get('measurement_stage') in ['rgb','h264','webrtc']:self.hub.measurement_stage=p['measurement_stage']
   if p.get('fault') in FAULTS:self.hub.set_fault(p['fault'])
   if p.get('profile') in ['high','balanced','low','emergency']:self.hub.adaptive=False;self.hub.profile=p['profile']
   if 'adaptive' in p:self.hub.adaptive=bool(p['adaptive']);self.hub.choose_profile()
   return web.json_response(self.hub.config())
  if path=='/api/telemetry' and r.method=='POST':
   p=await r.json();client=str(p.get('client','unknown'))[:80]
   for record in p.get('samples',[])[:200]:
    if record.get('stream') not in ['rgb','depth','h264','webrtc']:continue
    if not isinstance(record.get('boot'),str):continue
    self.telemetry[client].append(record)
   self.client_seen[client]=time.time()
   with (self.state/'browser-telemetry.jsonl').open('a') as log:log.write(json.dumps({'client':client,'samples':p.get('samples',[])[:200]})+'\n')
   (self.state/'browser-telemetry.jsonl').chmod(0o600)
   return web.json_response({'ok':True})
  if path=='/api/offer' and r.method=='POST':return await self.offer(r)
  if r.method!='GET':raise web.HTTPMethodNotAllowed(r.method,['GET'])
  if path=='/api/clock':return web.json_response({'server_ms':time.time()*1000})
  if path=='/api/state':
   data={k:v for k,v in self.current.items() if k!='fixture_packet'};data['media']=self.hub.snapshot();data['role']=role;data['video']=self.video_status();return web.json_response(wire_numbers(data))
  if path=='/api/metrics':return web.json_response(wire_numbers({'media':self.hub.snapshot(),'server':dict(self.counters),'clients':{c:list(v) for c,v in self.telemetry.items()},'client_seen':self.client_seen,'since':self.since}))
  if path=='/api/frame-meta':
   meta=self.frame_meta.get(int(r.query.get('seq','-1')))
   if not meta:raise web.HTTPNotFound()
   return web.json_response(meta)
  if path=='/api/depth/raw':
   item=self.hub.raw_depth.get(r.query.get('key',''))
   if not item:raise web.HTTPGone(text='Metric frame expired; no pixel distance guessed')
   meta,raw=item;return web.Response(body=raw,headers={'Content-Type':'application/octet-stream','X-Depth-Meta':json.dumps(meta,separators=(',',':'))})
  if path.startswith('/api/map/') and self.args.mission_url:
   from urllib.parse import quote
   import aiohttp
   rid=path.rsplit('/',1)[1].removesuffix('.png');token=Path(self.args.mission_token_file).expanduser().read_text().strip()
   async with aiohttp.ClientSession() as session:
    async with session.get(self.args.mission_url+'/api/v1/robots/'+quote(rid,safe='')+'/map.png',headers={'Authorization':'Bearer '+token},timeout=aiohttp.ClientTimeout(total=.5)) as response:
     if response.status!=200:raise web.HTTPNotFound()
     raw=await response.read()
     if not raw.startswith(b'\x89PNG'):raise web.HTTPBadGateway()
     return web.Response(body=raw,content_type='image/png')
  if path.startswith('/api/grid/'):
   name=path.rsplit('/',1)[1].removesuffix('.png');item=self.model.store.layers.get(name)
   if not item:raise web.HTTPNotFound()
   g=item[0];g.meta(name);return web.Response(body=g.png,content_type='image/png')
  if path in ['/api/v1/dashboard','/api/v1/status','/api/v1/frame.jpg']:
   # Separate legacy comparison upstream. The same viewer token authenticates it.
   if path.endswith('dashboard'):return web.json_response(wire_numbers(self.current['mission']))
   if path.endswith('status'):return web.json_response(self.video_status())
   p=self.hub.rgb.value
   if not p:raise web.HTTPNotFound()
   return web.Response(body=p.payload,content_type='image/jpeg',headers={'X-Frame-Seq':str(p.header['sequence']),'X-Frame-Stamp-Ns':p.header['stamp_ns']})
  raise web.HTTPNotFound()
 def video_status(self):
  if self.hub.mode=='LIVE':return {'ai_state':'NO_SENDER','fresh':False,'source':'UNAVAILABLE'}
  p=self.hub.rgb.value
  if not p:return {'ai_state':'NO_SENDER','fresh':False}
  h=p.header;age=(time.time()*1000-h['capture_wall_ms'])/1000
  return {'server_boot':self.hub.boot,'ai_state':'RECEIVING','fresh':age<1,'last_frame_age_s':age,'rx_fps':self.hub.config()['rgb_fps_cap'],'last_result':{'seq':h['sequence'],'frame_id':h['frame_id'],'stamp_ns':h['stamp_ns'],'age_s':age,'detections':h['detections'],'w':h['width'],'h':h['height']},'inference_mode':h['inference_mode'],'infer_ms_avg':h['inference_ms_original']}
 async def ws(self,r):
  if not self.role(r):raise web.HTTPUnauthorized()
  if not self.origin(r):raise web.HTTPForbidden()
  stream=r.match_info['stream']
  if stream not in ['rgb','depth','h264']:raise web.HTTPNotFound()
  if stream=='h264':self.hub.h264_enabled=True
  channel=getattr(self.hub,stream);ws=web.WebSocketResponse(heartbeat=3,compress=False);await ws.prepare(r);self.connections.add(ws);seen=0;lastboot=None
  ack=asyncio.Event();waiting=[None]
  async def receive_feedback():
   async for message in ws:
    if message.type==WSMsgType.TEXT:
     try:
      data=json.loads(message.data)
      if data.get('ack')==waiting[0]:ack.set()
     except (ValueError,TypeError):pass
   ack.set()
  reader=asyncio.create_task(receive_feedback())
  try:
   while not ws.closed:
    if self.hub.mode=='LIVE':await asyncio.sleep(.1);continue
    version,p=await channel.get(seen);self.counters['client_latest_skipped']+=max(0,version-seen-1);seen=version
    service_start=time.perf_counter()
    if not await self.hub.impairment(len(p.payload)):continue
    age=time.time()*1000-p.header['capture_wall_ms']
    if age>700:self.hub.observe_delivery((time.perf_counter()-service_start)*1000);self.counters['stale_frame_dropped']+=1;continue
    body=envelope(dict(p.header,send_wall_ms=time.time()*1000),p.payload)
    waiting[0]=p.header['sequence'];ack.clear();await asyncio.wait_for(ws.send_bytes(body),.5);await asyncio.wait_for(ack.wait(),1);self.hub.observe_delivery((time.perf_counter()-service_start)*1000);self.counters['ws_bytes']+=len(body);self.counters['ws_frames']+=1
  except (Exception,asyncio.CancelledError):pass
  finally:reader.cancel();await asyncio.gather(reader,return_exceptions=True);self.connections.discard(ws);await ws.close()
  return ws
 async def offer(self,r):
  if self.hub.mode=='LIVE':raise web.HTTPForbidden()
  p=await r.json();self.hub.h264_enabled=True;pc=RTCPeerConnection(RTCConfiguration(iceServers=[]));self.peers.add(pc);sender=pc.addTrack(SharedH264(self))
  codecs=[c for c in RTCRtpSender.getCapabilities('video').codecs if c.mimeType=='video/H264']
  pc.getTransceivers()[0].setCodecPreferences(codecs)
  original=sender.transport._send_rtp
  async def send_rtp(data):
   # Exact local RTP-datagram loss injection. No kernel/network configuration changed.
   from aiortc.rtp import is_rtcp
   f=FAULTS[self.hub.fault]
   if not is_rtcp(data) and self.hub.rng.random()<f['loss']:
    self.hub.metrics['rtp_datagrams_dropped']+=1;return
   await original(data);self.counters['webrtc_rtp_bytes']+=len(data)
  sender.transport._send_rtp=send_rtp
  @pc.on('connectionstatechange')
  async def changed():
   if pc.connectionState in ['failed','closed']:await pc.close();self.peers.discard(pc)
  await pc.setRemoteDescription(RTCSessionDescription(sdp=p['sdp'],type=p['type']));await pc.setLocalDescription(await pc.createAnswer());return web.json_response({'sdp':pc.localDescription.sdp,'type':pc.localDescription.type})

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=18310);parser.add_argument('--state',default='~/.local/state/capstone-dashboard-lab');parser.add_argument('--assets',required=True);parser.add_argument('--recorded');parser.add_argument('--inference');parser.add_argument('--mission-url');parser.add_argument('--mission-token-file');parser.add_argument('--adapter-snapshot');parser.add_argument('--mode',choices=['LIVE','REPLAY','SIMULATION'],default='SIMULATION');parser.add_argument('--baseline-url');parser.add_argument('--baseline-token-file');a=parser.parse_args()
 if a.port in [8080,18190,18210,18211]:raise SystemExit('Protected port refused')
 from urllib.parse import urlsplit
 if a.mission_url and (urlsplit(a.mission_url).hostname not in ['127.0.0.1','localhost','::1'] or not a.mission_token_file):raise SystemExit('Only explicitly configured loopback GET upstream allowed')
 a.baseline_token=Path(a.baseline_token_file).read_text().strip() if a.baseline_token_file else None;lab=Lab(a)
 @web.middleware
 async def policy(r,handler):
  try:res=await handler(r)
  except web.HTTPException as e:res=e
  res.headers.update({'Cache-Control':'no-store','X-Content-Type-Options':'nosniff','Content-Security-Policy':"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; media-src 'self' blob:; connect-src 'self' ws://127.0.0.1:*; frame-ancestors 'none'"})
  return res
 app=web.Application(middlewares=[policy],client_max_size=256*1024);app.on_startup.append(lab.startup);app.on_shutdown.append(lab.shutdown);app.on_cleanup.append(lab.cleanup)
 app.router.add_route('*','/api/{rest:.*}',lab.api);app.router.add_get('/health',lab.api);app.router.add_get('/ws/{stream}',lab.ws)
 async def static(r):
  name=r.match_info.get('name') or 'index.html'
  if name not in ['index.html','app.mjs','map.mjs','geometry.mjs','style.css']:raise web.HTTPNotFound()
  return web.FileResponse(ROOT/'web'/name)
 app.router.add_get('/',static);app.router.add_get('/{name}',static)
 print('Isolated lab http://127.0.0.1:'+str(a.port)+' — no robot connectivity',flush=True)
 web.run_app(app,host='127.0.0.1',port=a.port,print=None)
if __name__=='__main__':main()
