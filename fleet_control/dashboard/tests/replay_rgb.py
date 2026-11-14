"""Replay saved Gazebo JPEGs through the existing vlink/YOLO protocol; never a real-camera claim."""
import argparse,glob,json,math,socket,sys,threading,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--project',required=True);p.add_argument('--token-file',required=True);p.add_argument('--port',type=int,default=18220);a=p.parse_args()
sys.path.insert(0,a.project)
from vlink import protocol as P
from PIL import Image
files=sorted(Path(a.project,'assets/replay_frames').glob('*.jpg'));assert files
TOKEN=Path(a.token_file).read_text().strip();seq=0
while True:
 try:
  with socket.create_connection(('127.0.0.1',a.port),timeout=3) as conn:
   conn.settimeout(4);conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
   conn.sendall(P.pack({'type':'hello','proto':P.PROTO,'token':TOKEN,'robot_id':'gazebo-jpeg-replay','stream_id':'RECORDED_GAZEBO_REPLAY','codec':'jpeg'}));P.read_message(conn)
   stop=threading.Event()
   def receive():
    try:
     while not stop.is_set():P.read_message(conn)
    except Exception:stop.set()
   threading.Thread(target=receive,daemon=True).start()
   while not stop.is_set():
    file=files[(seq//5)%len(files)];payload=file.read_bytes()
    with Image.open(file) as image:w,h=image.size
    seq+=1;now=time.time_ns()
    conn.sendall(P.pack({'type':'frame','seq':seq,'frame_id':'RECORDED_GAZEBO_RGB_REPLAY_CLOCK','stamp_ns':now,'sent_wall_ns':now,'w':w,'h':h},payload))
    time.sleep(.2)
 except (OSError,ConnectionError,P.ProtocolError):time.sleep(1)
