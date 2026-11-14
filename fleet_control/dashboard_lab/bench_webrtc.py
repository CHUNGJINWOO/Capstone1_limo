"""Native loopback WebRTC receiver; distinct from browser compositor measurements."""
import argparse,asyncio,json,time
from pathlib import Path
import aiohttp
from aiortc import RTCPeerConnection,RTCSessionDescription,RTCConfiguration
from .media import percentiles
async def run(a):
 state=Path(a.state).expanduser();viewer=(state/'viewer.token').read_text().strip();lab=(state/'lab.token').read_text().strip();pc=RTCPeerConnection(RTCConfiguration(iceServers=[]));pc.addTransceiver('video',direction='recvonly');track_ready=asyncio.Future()
 @pc.on('track')
 def track(t):
  if not track_ready.done():track_ready.set_result(t)
 try:
  async with aiohttp.ClientSession() as s:
   async with s.post('http://127.0.0.1:18310/api/lab/control',json={'fault':'normal','adaptive':True},headers={'Authorization':'Bearer '+lab}) as r:assert r.status==200
   await pc.setLocalDescription(await pc.createOffer())
   async with s.post('http://127.0.0.1:18310/api/offer',json={'sdp':pc.localDescription.sdp,'type':'offer'},headers={'Authorization':'Bearer '+viewer}) as r:answer=await r.json()
   await pc.setRemoteDescription(RTCSessionDescription(**answer));track=await asyncio.wait_for(track_ready,5);started=time.monotonic();rows=[]
   while time.monotonic()-started<a.seconds:
    frame=await asyncio.wait_for(track.recv(),4);b=frame.to_ndarray(format='bgr24');seq=0;stamp=0
    for i in range(64):
     bit=int(b[min(4,b.shape[0]-1),min(b.shape[1]-1,int((i+.5)*b.shape[1]/64)),2]>128)
     if i<32:seq=seq*2+bit
     else:stamp=stamp*2+bit
    latency=((int(time.time()*1000)&0xffffffff)-stamp)&0xffffffff
    if latency<2000:rows.append({'sequence':seq,'capture_to_native_decode_ms':latency})
   elapsed=time.monotonic()-started;out={'native_fps':len(rows)/elapsed,'seconds':elapsed,'latency':percentiles(r['capture_to_native_decode_ms'] for r in rows),'samples':rows};Path(a.out).write_text(json.dumps(out,indent=2));print({k:v for k,v in out.items() if k!='samples'})
 finally:await pc.close()
def main():
 p=argparse.ArgumentParser();p.add_argument('--state',default='~/.local/state/capstone-dashboard-lab');p.add_argument('--seconds',type=float,default=8);p.add_argument('--out',required=True);a=p.parse_args();asyncio.run(run(a))
if __name__=='__main__':main()
