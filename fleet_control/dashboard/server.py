"""Loopback dashboard gateway: GET-only upstreams and a separate authenticated telemetry sink."""
import argparse, collections, hashlib, http.cookies, json, os, secrets, signal, threading, time
import urllib.error, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from .core import Store

ROOT=Path(__file__).parent/'web'

def read_token(path):
    p=Path(path).expanduser()
    if p.stat().st_mode & 0o077:raise ValueError('token file must have mode 0600')
    token=p.read_text().strip()
    if len(token)<24:raise ValueError('token too short')
    return token

def get(url,token):
    request=urllib.request.Request(url,headers={'Authorization':'Bearer '+token})
    with urllib.request.urlopen(request,timeout=2) as r:
        data=r.read(12*1024*1024+1)
        if len(data)>12*1024*1024:raise ValueError('upstream response too large')
        return data,dict(r.headers)

def key(result,boot):
    return None if not result else (boot,result.get('seq'),result.get('frame_id'),result.get('stamp_ns'))

class Gateway:
    def __init__(self,cfg):
        self.cfg=cfg;self.profile=cfg.get('profile','live');self.store=Store(self.profile)
        self.viewer=read_token(cfg['viewer_token_file']);self.bridge=read_token(cfg['bridge_token_file'])
        if self.viewer==self.bridge:raise ValueError('viewer and bridge credentials must differ')
        self.mm_token=read_token(cfg['mission_token_file']) if cfg.get('mission_token_file') else None
        self.ai_token=read_token(cfg['ai_token_file']) if cfg.get('ai_token_file') else None
        self.lock=threading.RLock();self.stop=threading.Event();self.sessions={};self.failures=collections.deque(maxlen=100)
        self.mm={'data':{'robots':[],'missions':[],'events':[],'server':{}},'connected':False,'received':0}
        self.video={'status':None,'frame':None,'connected':False,'paired':False,'received':0}
        self.maps={};self.metrics={'mm_requests':0,'video_requests':0,'upstream_bytes':0,'browser_bytes':0,'requests':0,'pair_mismatch':0}
        self.cookie='capstone_dash_'+str(cfg['port'])
    def run_workers(self):
        for fn in [self.poll_mm,self.poll_video]:threading.Thread(target=fn,daemon=True).start()
    def poll_mm(self):
        while not self.stop.is_set():
            if self.mm_token:
                try:
                    data,_=get(self.cfg['mission_url']+'/api/v1/dashboard',self.mm_token)
                    dashboard=json.loads(data)
                    for robot in dashboard.get('robots',[]):
                        rid=robot['robot_id'];version=robot.get('map_version')
                        if version and (rid not in self.maps or self.maps[rid]['version']!=version):
                            from urllib.parse import quote
                            png,h=get(self.cfg['mission_url']+'/api/v1/robots/'+quote(rid,safe='')+'/map.png',self.mm_token)
                            if not png.startswith(b'\x89PNG\r\n\x1a\n'):raise ValueError('invalid map image')
                            with self.lock:self.maps[rid]={'version':version,'png':png}
                    with self.lock:
                        self.mm={'data':dashboard,'connected':True,'received':time.monotonic()}
                        self.metrics['mm_requests']+=1;self.metrics['upstream_bytes']+=len(data)
                except Exception as exc:
                    with self.lock:self.mm['connected']=False;self.mm['error']=type(exc).__name__
            self.stop.wait(.5)
    def poll_video(self):
        while not self.stop.is_set():
            if self.ai_token:
                try:
                    b,_=get(self.cfg['ai_url']+'/api/v1/status',self.ai_token);s=json.loads(b)
                    with self.lock:old=self.video.get('pair_key')
                    result=s.get('last_result');ident=key(result,s.get('server_boot'))
                    frame=None;paired=False
                    if ident and ident!=old:
                        frame,headers=get(self.cfg['ai_url']+'/api/v1/frame.jpg',self.ai_token)
                        b2,_=get(self.cfg['ai_url']+'/api/v1/status',self.ai_token);after=json.loads(b2)
                        seq=headers.get('X-Frame-Seq') or headers.get('x-frame-seq')
                        paired=(str(seq)==str(result.get('seq')) and ident==key(after.get('last_result'),after.get('server_boot')))
                        if not paired:self.metrics['pair_mismatch']+=1;frame=None
                    with self.lock:
                        if paired:self.video.update(frame=frame,pair_key=ident,paired=True,frame_result=result)
                        self.video.update(status=s,connected=True,received=time.monotonic())
                        self.metrics['video_requests']+=1;self.metrics['upstream_bytes']+=len(b)+(len(frame) if frame else 0)
                except Exception as exc:
                    with self.lock:self.video.update(connected=False,error=type(exc).__name__)
            self.stop.wait(.2)
    def snapshot(self):
        with self.lock:
            now=time.monotonic();v=self.video;s=v.get('status') or {};result=v.get('frame_result')
            current=key(s.get('last_result'),s.get('server_boot'))
            paired=v.get('paired',False) and v.get('pair_key')==current
            source_age=(result or {}).get('age_s')
            live=bool(v.get('connected') and paired and s.get('ai_state')=='RECEIVING' and
                      s.get('last_frame_age_s') is not None and s['last_frame_age_s']<=2 and
                      (s.get('last_result') or {}).get('age_s',999)<=2)
            video={k:val for k,val in s.items() if k!='last_result'}
            video.update(connected=v.get('connected',False),paired=paired,fresh=live,
                         result=s.get('last_result') if paired else None,
                         frame_id=hashlib.sha256(repr(v.get('pair_key')).encode()).hexdigest()[:16] if v.get('frame') else None)
            return {'profile':self.profile,'read_only':True,'server_time':time.time(),
                    'mission':dict(self.mm['data'],connected=self.mm['connected'],cache_age_s=round(now-self.mm['received'],2) if self.mm['received'] else None),
                    'video':video,'visualization':self.store.snapshot(),'gateway_metrics':dict(self.metrics),
                    'rmf':{'status':'UNAVAILABLE','reason':'No live RMF task-state endpoint configured'},
                    'capabilities':{'mission_create':False,'mission_cancel':False,'motor_control':False,'emergency_stop':False}}

def wire_numbers(value, name=''):
    # Nanoseconds exceed JS Number's 53-bit integer range. Preserve exact keys.
    if isinstance(value,dict):return {k:wire_numbers(v,k) for k,v in value.items()}
    if isinstance(value,list):return [wire_numbers(v,name) for v in value]
    if isinstance(value,int) and name.endswith('_ns'):return str(value)
    return value

class Handler(BaseHTTPRequestHandler):
    server_version='CapstoneVisualization/1'
    def log_message(self,*args):pass
    @property
    def app(self):return self.server.app
    def send(self,code,body=b'',mime='application/json',headers=None):
        if not isinstance(body,bytes):body=json.dumps(wire_numbers(body),ensure_ascii=False,allow_nan=False).encode()
        self.send_response(code)
        for k,v in {'Content-Type':mime,'Content-Length':str(len(body)),'Cache-Control':'no-store',
                    'X-Content-Type-Options':'nosniff','Referrer-Policy':'no-referrer',
                    'Content-Security-Policy':"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'; object-src 'none'",**(headers or {})}.items():self.send_header(k,v)
        self.end_headers()
        try:self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError):pass
        with self.app.lock:self.app.metrics['browser_bytes']+=len(body);self.app.metrics['requests']+=1
    def authorized(self):
        auth=self.headers.get('Authorization','')
        if auth.startswith('Bearer ') and secrets.compare_digest(auth[7:],self.app.viewer):return True
        try:
            cookies=http.cookies.SimpleCookie(self.headers.get('Cookie',''));sid=cookies[self.app.cookie].value
            return self.app.sessions.get(sid,0)>time.monotonic()
        except (KeyError,http.cookies.CookieError):return False
    def origin_ok(self):
        origin=self.headers.get('Origin')
        return not origin or origin=='http://'+self.headers.get('Host','')
    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/health':return self.send(200,{'status':'ok','profile':self.app.profile,'read_only':True})
        assets={'/':'index.html','/app.mjs':'app.mjs','/map.mjs':'map.mjs','/geometry.mjs':'geometry.mjs','/style.css':'style.css'}
        if path in assets:
            p=ROOT/assets[path];mime='text/html; charset=utf-8' if p.suffix=='.html' else ('text/css' if p.suffix=='.css' else 'text/javascript')
            return self.send(200,p.read_bytes(),mime)
        if not self.authorized():return self.send(401,{'error':'unauthorized'})
        if path=='/api/state':return self.send(200,self.app.snapshot())
        if path=='/api/video.jpg':
            with self.app.lock:frame=self.app.video.get('frame');ident=self.app.video.get('pair_key')
            if not frame:return self.send(404,{'error':'no matched video frame'})
            return self.send(200,frame,'image/jpeg',{'X-Frame-Seq':str(ident[1]),'X-Frame-Stamp-Ns':str(ident[3])})
        if path=='/api/depth':
            with self.app.store.lock:depth=self.app.store.depth
            if not depth:return self.send(404,{'error':'depth unavailable'})
            return self.send(200,depth[0])
        if path.startswith('/api/grid/') and path.endswith('.png'):
            name=path[len('/api/grid/'):-4]
            with self.app.store.lock:
                item=self.app.store.layers.get(name)
                if not item:return self.send(404,{'error':'grid unavailable'})
                g=item[0];g.meta(name);png=g.png
            return self.send(200,png,'image/png')
        if path.startswith('/api/map/') and path.endswith('.png'):
            rid=path[len('/api/map/'):-4]
            with self.app.lock:item=self.app.maps.get(rid)
            if not item:return self.send(404,{'error':'map unavailable'})
            return self.send(200,item['png'],'image/png')
        return self.send(404,{'error':'not found'})
    def do_POST(self):
        path=urlsplit(self.path).path
        if path not in ['/api/login','/api/logout','/api/bridge']:return self.send(405,{'error':'read_only; robot commands are disabled'})
        if not self.origin_ok():return self.send(403,{'error':'origin rejected'})
        try:
            length=int(self.headers.get('Content-Length','0'))
            if length<0 or length>12*1024*1024:return self.send(413,{'error':'payload too large'})
            p=json.loads(self.rfile.read(length) or b'{}',parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite JSON')))
        except (ValueError,UnicodeDecodeError):return self.send(400,{'error':'invalid JSON'})
        if not isinstance(p,dict):return self.send(400,{'error':'object required'})
        if path=='/api/login':
            now=time.monotonic()
            if sum(t>now-60 for t in self.app.failures)>=10:return self.send(429,{'error':'retry later'})
            if not isinstance(p.get('token'),str) or not secrets.compare_digest(p['token'],self.app.viewer):
                self.app.failures.append(now);return self.send(401,{'error':'unauthorized'})
            sid=secrets.token_urlsafe(32)
            with self.app.lock:
                self.app.sessions={k:v for k,v in self.app.sessions.items() if v>now}
                self.app.sessions[sid]=now+8*3600
            return self.send(200,{'role':'viewer','read_only':True},'application/json',
                             {'Set-Cookie':self.app.cookie+'='+sid+'; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800'})
        if path=='/api/logout':
            try:
                c=http.cookies.SimpleCookie(self.headers.get('Cookie',''))
                with self.app.lock:self.app.sessions.pop(c[self.app.cookie].value,None)
            except (KeyError,http.cookies.CookieError):pass
            return self.send(200,{'ok':True},headers={'Set-Cookie':self.app.cookie+'=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0'})
        auth=self.headers.get('Authorization','')
        if not auth.startswith('Bearer ') or not secrets.compare_digest(auth[7:],self.app.bridge):return self.send(401,{'error':'bridge unauthorized'})
        try:self.app.store.ingest(p)
        except (ValueError,KeyError,TypeError,OverflowError) as exc:
            self.app.store.counters['rejected']+=1;return self.send(400,{'error':'invalid telemetry','reason':str(exc)[:120]})
        return self.send(200,{'ok':True,'sequence':self.app.store.sequence})
    def do_PUT(self):self.send(405,{'error':'read_only'})
    def do_DELETE(self):self.send(405,{'error':'read_only'})

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--config',required=True);a=ap.parse_args()
    cfg=json.loads(Path(a.config).expanduser().read_text())
    if cfg.get('bind','127.0.0.1')!='127.0.0.1':raise SystemExit('Visualization is loopback-only')
    if cfg['port'] in [8080,18190]:raise SystemExit('Protected port')
    app=Gateway(cfg);app.run_workers();srv=ThreadingHTTPServer(('127.0.0.1',cfg['port']),Handler);srv.app=app;srv.daemon_threads=True
    def stop(*_):app.stop.set();threading.Thread(target=srv.shutdown,daemon=True).start()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    print('dashboard http://127.0.0.1:'+str(cfg['port'])+' profile='+app.profile+' READ_ONLY',flush=True)
    try:srv.serve_forever()
    finally:app.stop.set();srv.server_close()
if __name__=='__main__':main()
