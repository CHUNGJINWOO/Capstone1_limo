"""Validated visualization data, OccupancyGrid updates and metric depth conversion."""
import base64, hashlib, io, math, threading, time, zlib
import numpy as np
from PIL import Image

KINDS = {'ROS_LIVE', 'SYNTHETIC_ROS_TEST', 'GAZEBO_REPLAY'}
GRID_NAMES = {'map', 'global_costmap', 'local_costmap'}
PATH_NAMES = {'global_plan', 'local_plan', 'trail', 'frontiers', 'selected_frontier'}

def number(v):
    if isinstance(v, bool) or not isinstance(v, (float,int)) or not math.isfinite(v):
        raise ValueError('finite number required')
    return float(v)

def pose(p):
    return {k:number(p[k]) for k in ('x','y','yaw')}

def compose(a,b):
    c,s=math.cos(a['yaw']),math.sin(a['yaw'])
    return {'x':a['x']+c*b['x']-s*b['y'],'y':a['y']+s*b['x']+c*b['y'],'yaw':a['yaw']+b['yaw']}

def png_rgba(pixels):
    out=io.BytesIO();Image.fromarray(pixels.astype(np.uint8),'RGBA').save(out,format='PNG')
    return out.getvalue()

def unpack(data, expected):
    raw=base64.b64decode(data,validate=True)
    decoder=zlib.decompressobj();out=decoder.decompress(raw,expected+1)
    if len(out)!=expected or not decoder.eof or decoder.unused_data:raise ValueError('compressed shape mismatch')
    return out

def pack(data):return base64.b64encode(zlib.compress(data,3)).decode()

def grid_png(cells, costmap):
    a=np.asarray(cells,dtype=np.int16)
    rgba=np.zeros((*a.shape,4),np.uint8)
    if costmap:
        rgba[a<0]=[112,128,146,65]
        inflation=(a>0)&(a<99)
        rgba[inflation,0]=238;rgba[inflation,1]=173;rgba[inflation,2]=65
        rgba[inflation,3]=(65+a[inflation]*1.3).astype(np.uint8)
        rgba[a>=99]=[247,91,105,235]
    else:
        rgba[:]=[184,197,209,255];rgba[a==0]=[224,232,238,255]
        rgba[a>=65]=[51,68,84,255];rgba[(a>0)&(a<65)]=[149,165,180,255]
        rgba[a<0]=[25,37,50,255]
    return png_rgba(np.flipud(rgba))  # ROS row0=bottom; PNG row0=top

class Grid:
    def __init__(self, value):
        self.width=int(value['width']);self.height=int(value['height'])
        if self.width<=0 or self.height<=0 or self.width*self.height>4_000_000:raise ValueError('grid size')
        self.resolution=number(value['resolution'])
        if not 0<self.resolution<=10:raise ValueError('resolution')
        self.origin=pose(value['origin']);self.frame=str(value['frame_id'])[:128]
        self.transform=pose(value['map_from_frame']) if value.get('map_from_frame') is not None else None
        if self.frame=='map' and self.transform is None:self.transform={'x':0.,'y':0.,'yaw':0.}
        raw=unpack(value['cells_zlib'],self.width*self.height)
        self.cells=np.frombuffer(raw,dtype=np.int8).reshape(self.height,self.width).copy()
        if np.any(self.cells>100) or np.any(self.cells < -1):raise ValueError('OccupancyGrid value out of range')
        self.stamp_ns=int(value.get('stamp_ns',0));self.source_age_s=number(value['source_age_s']) if value.get('source_age_s') is not None else None
        self.topic=str(value.get('topic',''))[:256];self.revision=0;self.png=None
    def update(self, patch):
        x,y,w,h=[int(patch[k]) for k in ('x','y','width','height')]
        if x<0 or y<0 or w<=0 or h<=0 or x+w>self.width or y+h>self.height:raise ValueError('patch bounds')
        if patch.get('frame_id',self.frame)!=self.frame:raise ValueError('patch frame mismatch')
        raw=unpack(patch['cells_zlib'],w*h);v=np.frombuffer(raw,dtype=np.int8).reshape(h,w)
        if np.any(v>100) or np.any(v < -1):raise ValueError('patch values')
        self.cells[y:y+h,x:x+w]=v;self.stamp_ns=int(patch.get('stamp_ns',self.stamp_ns));self.png=None
    def meta(self, name):
        if self.png is None:self.png=grid_png(self.cells,name!='map')
        return {'width':self.width,'height':self.height,'resolution':self.resolution,'origin':self.origin,
                'frame_id':self.frame,'map_from_frame':self.transform,'stamp_ns':self.stamp_ns,
                'source_age_s_at_capture':self.source_age_s,'topic':self.topic,
                'renderable':self.transform is not None,'tf_status':'OK' if self.transform is not None else 'UNAVAILABLE',
                'image_id':hashlib.sha256(self.png).hexdigest()[:16]}

def metric_depth(data, *, width, height, step, encoding, bigendian=False, scale_m=.001, max_width=320):
    if width<=0 or height<=0 or width*height>4_000_000:raise ValueError('depth shape')
    if encoding not in ('16UC1','32FC1'):raise ValueError('unsupported depth encoding')
    size=2 if encoding=='16UC1' else 4
    if step<width*size or len(data)!=step*height:raise ValueError('depth step/data mismatch')
    if encoding=='16UC1' and not 0<number(scale_m)<=1:raise ValueError('depth scale')
    dtype=np.dtype(('>' if bigendian else '<')+('u2' if size==2 else 'f4'))
    depth=np.ndarray((height,width),dtype=dtype,buffer=data,strides=(step,size)).astype(np.float32)
    if encoding=='16UC1':depth*=scale_m
    invalid=~np.isfinite(depth)|(depth<=0);depth[invalid]=np.nan
    stride=max(1,math.ceil(width/max_width));depth=depth[::stride,::stride].astype('<f4')
    valid=depth[np.isfinite(depth)]
    return {'width':depth.shape[1],'height':depth.shape[0],'source_width':width,'source_height':height,
            'sample_stride':stride,'encoding':encoding,'scale_m':scale_m if size==2 else 1.0,
            'unit':'m','values_f32le_zlib':pack(depth.tobytes()),'invalid':'NaN/zero/nonfinite',
            'valid_min_m':float(valid.min()) if valid.size else None,'valid_max_m':float(valid.max()) if valid.size else None,
            'valid_fraction':float(valid.size/depth.size),'rgb_registered':False}

def fresh_status(received_age, source_age, renderable):
    if not renderable:return 'UNAVAILABLE'
    if received_age>3 or (source_age is not None and source_age>3):return 'STALE'
    if source_age is None or source_age<-.2:return 'TIME_UNVERIFIED'
    return 'OK'

class Store:
    def __init__(self, profile):
        self.profile=profile;self.lock=threading.RLock();self.layers={};self.paths={};self.depth=None
        self.last=0.;self.kind=None;self.session=None;self.retired=set();self.sequence=-1;self.counters={'received':0,'rejected':0,'out_of_order':0}
    def ingest(self,p):
        kind=p['kind'];session=str(p['session']);seq=int(p['sequence'])
        if kind not in KINDS or (self.profile=='live')!=(kind=='ROS_LIVE'):raise ValueError('live/test source isolation')
        with self.lock:
            if session in self.retired or (session==self.session and seq<=self.sequence):
                self.counters['out_of_order']+=1;raise ValueError('old visualization packet')
            grids={}
            for name, value in p.get('grids',{}).items():
                if name not in GRID_NAMES:raise ValueError('grid name')
                grids[name]=Grid(value)
            paths={}
            for name,value in p.get('paths',{}).items():
                if name not in PATH_NAMES:raise ValueError('path name')
                points=value.get('points',[])
                if len(points)>10000:raise ValueError('too many path points')
                pts=[[number(x),number(y)] for x,y in points]
                paths[name]=dict(value,points=pts)
                if value.get('frame_id')!='map' and value.get('renderable') is True:raise ValueError('path not transformed to map')
            depth=p.get('depth')
            if depth is not None:
                w,h=int(depth['width']),int(depth['height'])
                if w<=0 or h<=0 or w*h>320*320:raise ValueError('depth size')
                raw=unpack(depth['values_f32le_zlib'],w*h*4)
                values=np.frombuffer(raw,dtype='<f4')
                if np.isinf(values).any() or np.any(values[np.isfinite(values)]<0):raise ValueError('invalid metric depth')
            if self.session and session!=self.session:self.retired.add(self.session)
            if session!=self.session:self.layers={};self.paths={};self.depth=None
            now=time.monotonic()
            for name,g in grids.items():
                old=self.layers.get(name)
                if old and old[0].cells.shape==g.cells.shape and np.array_equal(old[0].cells,g.cells):g.png=old[0].png
                self.layers[name]=(g,now)
            for name,v in paths.items():self.paths[name]=(v,now)
            if depth is not None:self.depth=(depth,now)
            self.kind=kind;self.session=session;self.sequence=seq;self.last=now;self.counters['received']+=1
            self.meta={k:p.get(k) for k in ['robot_id','ros_time_ns','clock_mode','explore','camera_info','metrics','unavailable','robot_pose']}
    def snapshot(self):
        with self.lock:
            now=time.monotonic();layers={}
            for name,(g,t) in self.layers.items():
                layers[name]=dict(g.meta(name),receive_age_s=round(now-t,3),status=fresh_status(now-t,g.source_age_s,g.transform is not None))
            paths={k:dict(v,receive_age_s=round(now-t,3),status=fresh_status(now-t,v.get('source_age_s'),bool(v.get('renderable')))) for k,(v,t) in self.paths.items()}
            return {'kind':self.kind,'profile':self.profile,'connected':bool(self.last and now-self.last<3),
                    'receive_age_s':round(now-self.last,3) if self.last else None,'sequence':self.sequence,
                    'layers':layers,'paths':paths,'depth':None if not self.depth else
                    {k:v for k,v in dict(self.depth[0],receive_age_s=round(now-self.depth[1],3),status=fresh_status(now-self.depth[1],self.depth[0].get('source_age_s'),True)).items() if k!='values_f32le_zlib'},
                    'dynamic_obstacles':{'status':'UNIMPLEMENTED','reason':'2D YOLO is not a map-frame tracked position'},
                    'counters':dict(self.counters),**getattr(self,'meta',{})}
