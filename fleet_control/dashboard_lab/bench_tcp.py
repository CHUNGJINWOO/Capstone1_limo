"""Native loopback TCP codec measurement, separate from browser presentation metrics."""
import argparse,json,socket,struct,time,urllib.request
from pathlib import Path
import av,cv2,numpy as np
from .media import decode_envelope,percentiles

def read_exact(s,n):
 b=bytearray()
 while len(b)<n:
  v=s.recv(n-len(b))
  if not v:raise EOFError()
  b.extend(v)
 return bytes(b)
def main():
 p=argparse.ArgumentParser();p.add_argument('--state',default='~/.local/state/capstone-dashboard-lab');p.add_argument('--seconds',type=float,default=7);p.add_argument('--out',required=True);a=p.parse_args();S=Path(a.state).expanduser();viewer=(S/'viewer.token').read_text().strip();lab=(S/'lab.token').read_text().strip();reports=[]
 def control(**values):
  req=urllib.request.Request('http://127.0.0.1:18310/api/lab/control',data=json.dumps(values).encode(),headers={'Authorization':'Bearer '+lab,'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=3) as r:return json.load(r)
 for profile,codec in [('high','jpeg'),('balanced','jpeg'),('low','jpeg'),('emergency','jpeg'),('balanced','h264')]:
  control(fault='normal',profile=profile);stats=[];decoder=av.CodecContext.create('h264','r');ready=False;started=time.monotonic();total=0
  with socket.create_connection(('127.0.0.1',18313),timeout=3) as s:
   s.settimeout(3);s.sendall((json.dumps({'token':viewer,'codec':codec})+'\n').encode())
   while time.monotonic()-started<a.seconds:
    body=read_exact(s,struct.unpack('!I',read_exact(s,4))[0]);h,data=decode_envelope(body);total+=len(body);receive=time.time()*1000;begin=time.perf_counter()
    if codec=='jpeg':image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR);assert image is not None
    else:
     if not ready and not h['keyframe']:continue
     ready=True
     try:frames=decoder.decode(av.Packet(data))
     except av.FFmpegError:ready=False;continue
     if not frames:continue
    decoded=time.time()*1000
    if decoded-h['capture_wall_ms']>700:continue
    stats.append({'sequence':h['sequence'],'encode_ms':h.get('h264_encode_ms',h['encode_ms']),'encode_to_receive_ms':receive-h['encoded_wall_ms'],'decode_ms':(time.perf_counter()-begin)*1000,'capture_to_native_decode_ms':decoded-h['capture_wall_ms'],'bytes':len(body)})
  elapsed=time.monotonic()-started;summary={'profile':profile,'codec':codec,'seconds':round(elapsed,3),'native_fps':round(len(stats)/elapsed,3),'payload_kB_s':round(total/elapsed/1000,3),'samples':stats}
  for name in ['encode_ms','encode_to_receive_ms','decode_ms','capture_to_native_decode_ms']:summary[name]=percentiles(row[name] for row in stats)
  reports.append(summary);print(profile,codec,summary['native_fps'],summary['payload_kB_s'],flush=True)
 control(fault='normal',adaptive=True);Path(a.out).write_text(json.dumps(reports,indent=2))
if __name__=='__main__':main()
