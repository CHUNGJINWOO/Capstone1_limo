"""Original gateway/UI behavior with display telemetry only, separate loopback port."""
import argparse,collections,json,threading
from http.server import ThreadingHTTPServer
from pathlib import Path
import dashboard.server as original
SAMPLES=collections.deque(maxlen=10000);LOCK=threading.RLock()
class Handler(original.Handler):
 def do_GET(self):
  if self.path=='/api/benchmark':
   if not self.authorized():return self.send(401,{'error':'unauthorized'})
   with LOCK:return self.send(200,list(SAMPLES))
  return super().do_GET()
 def do_POST(self):
  if self.path=='/api/benchmark':
   if not self.authorized() or not self.origin_ok():return self.send(403,{'error':'forbidden'})
   try:
    n=int(self.headers.get('Content-Length','0'));assert 0<n<4096;p=json.loads(self.rfile.read(n));assert p['stream'] in ['baseline_rgb','baseline_depth']
    with LOCK:SAMPLES.append(p)
    return self.send(200,{'ok':True})
   except Exception:return self.send(400,{'error':'invalid sample'})
  return super().do_POST()
def main():
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);a=p.parse_args();c=json.loads(Path(a.config).read_text());assert c['port']==18311
 original.ROOT=Path(__file__).parent/'baseline_web';app=original.Gateway(c);app.run_workers();s=ThreadingHTTPServer(('127.0.0.1',18311),Handler);s.app=app;s.daemon_threads=True
 print('Legacy comparison http://127.0.0.1:18311/',flush=True)
 try:s.serve_forever()
 finally:app.stop.set();s.server_close()
if __name__=='__main__':main()
