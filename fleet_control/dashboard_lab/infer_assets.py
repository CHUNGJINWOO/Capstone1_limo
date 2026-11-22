"""Optional offline CPU YOLO preprocessing; never presented as live inference FPS."""
import argparse,hashlib,json,time
from pathlib import Path
import cv2,torch
from ultralytics import YOLO
p=argparse.ArgumentParser();p.add_argument('--assets',required=True);p.add_argument('--weights',required=True);p.add_argument('--out',required=True);a=p.parse_args()
torch.set_num_threads(2);model=YOLO(a.weights);results={}
for path in sorted(Path(a.assets).glob('*.jpg')):
 image=cv2.imread(str(path));start=time.perf_counter();r=model.predict(image,device='cpu',imgsz=320,verbose=False)[0];d=[]
 for box in r.boxes:
  x1,y1,x2,y2=map(float,box.xyxy[0].tolist());d.append({'cls':r.names[int(box.cls[0])],'conf':float(box.conf[0]),'x1':x1,'x2':x2,'y1':y1,'y2':y2})
 results[hashlib.sha256(path.read_bytes()).hexdigest()]={'detections':d,'infer_ms':round((time.perf_counter()-start)*1000,3),'source_file':path.name,'mode':'PRECOMPUTED_CPU_REPLAY'}
Path(a.out).write_text(json.dumps(results,indent=2));print('Precomputed CPU YOLO saved assets:',len(results))
