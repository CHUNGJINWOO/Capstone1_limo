"""Loopback-only UI fixture. Temporary auth/DB; no ROS, robot or operating server.
Run from any directory: python3 dashboard_preview.py [--info-file /tmp/ui-info.json].
All robot/map values are synthetic test data, identified by UI-FIXTURE robot IDs.
"""
import argparse
import base64
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import urllib.request
import zlib
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.map_registry import MapRegistry
from fleet_server.service import Fleet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--info-file', default='/tmp/limo-dashboard-preview.json')
    args = parser.parse_args()
    if Path(args.info_file).exists():
        raise SystemExit('info-file already exists; choose a new --info-file path')
    tmp = tempfile.TemporaryDirectory(prefix='limo-dashboard-ui-')
    root = Path(tmp.name)
    ids = ['UI-FIXTURE-01', 'UI-FIXTURE-02']
    init_tokens(str(root), str(root/'tokens'), ids)
    db = DB(str(root/'fleet.db'))
    fleet = Fleet(db, SimpleNamespace(offline_after_s=3, command_ttl_s=120, resend_after_s=1))
    registry = MapRegistry(str(root/'registry.db'), str(root/'maps'))
    registry.create_facility('test', 'UI TEST / SYNTHETIC')
    registry.create_building('test-building', 'test', '테스트 건물')
    registry.create_floor('test-floor', 'test-building', 'L1')
    w,h=160,120
    pixels = bytearray([254]*(w*h))
    for y in range(h):
        for x in range(w):
            if x<4 or y<4 or x>w-5 or y>h-5 or (x in range(46,50) and y not in range(45,66)) or (x in range(104,108) and y not in range(70,90)) or (y in range(54,58) and x>108): pixels[y*w+x]=0
    yaml=b'image: map.pgm\nresolution: 0.05\norigin: [-4.0, -3.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
    registered=registry.register_map_bundle(floor_id='test-floor',label='SYNTHETIC-UNVERIFIED',yaml_bytes=yaml,pgm_bytes=b'P5\n160 120\n255\n'+pixels,source_mode='SIM_FIXTURE',actor='ui-test')
    def chunk(k,d): return struct.pack('>I',len(d))+k+d+struct.pack('>I',zlib.crc32(k+d)&0xffffffff)
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(b''.join(b'\0'+pixels[y*w:(y+1)*w] for y in range(h))))+chunk(b'IEND',b'')
    server = Servers(fleet, Auth(str(root)), ['127.0.0.1'], 0, map_registry=registry)
    port=server.srvs[0].server_address[1]
    url=f'http://127.0.0.1:{port}'
    threading.Thread(target=server.serve_forever,daemon=True).start()
    def post(path, rid, body):
        token=(root/'tokens'/f'agent-{rid}.token').read_text().strip()
        req=urllib.request.Request(url+'/api/v1'+path,json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=3) as res:return json.load(res)
    ex={'active':None,'nav2_goals':{'known':True,'received_status_array':True,'active_count':0,'age_s':0}}
    for rid in ids:
        post('/agent/hello',rid,{'robot_id':rid,'agent':{'boot_id':'ui-test-boot','protocol_version':2,'safety_contract_version':1,'mode':'sim','simulated':True,'missions_enabled':True}})
        post('/agent/reconcile',rid,{'robot_id':rid,'agent_boot_id':'ui-test-boot','journal':{},'journal_ok':True,'execution':ex})
        post('/agent/map',rid,{'robot_id':rid,'agent_boot_id':'ui-test-boot','version':1,'meta':{'width':w,'height':h,'resolution':.05,'origin_x':-4,'origin_y':-3,'origin_yaw':0,'frame_id':'map'},'png_base64':base64.b64encode(png).decode()})
    scenario=root/'scenario.txt';scenario.write_text('normal')
    info={'url':url,'runtime':str(root),'scenario':str(scenario),'map_version':registered['map_version']['map_version_id'],'token_dir':str(root/'tokens')}
    info_path=Path(args.info_file)
    info_path.write_text(json.dumps(info));info_path.chmod(0o600)
    print(f'ISOLATED SYNTHETIC UI: {url}\nRuntime information: {info_path}\nLogin token: {root}/tokens/operator.token',flush=True)
    seq=0
    try:
        while True:
            seq+=1; mode=scenario.read_text().strip()
            for index,rid in enumerate(ids):
                if mode=='offline' and index==0: continue
                state={'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'fixture-map','localization_map_version_id':'fixture-map','map_verified':mode!='unverified','localization_status':'CONFIRMED','execution':ex,'pose':{'status':'OK','frame_id':'map','x':-.8+index,'y':-.4,'yaw_rad':.6},'battery':{'voltage_v':12.2,'status':'VOLTAGE_ONLY'},'lidar':{'status':'OK','age_s':0},'depth_camera':{'status':'OK','age_s':0},'map':{'status':'STATIC'},'safety_gate':{'boot_id':'fixture-gate','status':'NORMAL','age_s':0,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True,'rx_fresh':True,'source_fresh':True,'motor_path':{'status':'SIM_GATED'}}}
                if mode=='fault':state['safety_gate'].update(status='FAULT',fault_latched=True,cmd_blocked=True,movement_permitted=False,fault_reasons=['UI_FIXTURE_SENSOR_TIMEOUT'])
                if mode=='gate-stale':state['safety_gate'].update(status='STALE',age_s=10,rx_fresh=False,movement_permitted=False)
                if mode=='no-attestation':state['safety_gate'].update(movement_permitted=False,movement_block_reasons=['GATE_ATTESTATION_UNAVAILABLE'])
                post('/agent/state',rid,{'robot_id':rid,'agent_boot_id':'ui-test-boot','seq':seq,'state':state})
                if mode=='reconciliation':
                    with fleet.lock: fleet.robots[rid]['reconciliation_required']=True
            time.sleep(.35)
    except KeyboardInterrupt: pass
    finally:
        server.stop_evt.set()
        # Temporary resources belong to this fixture only. No production data is read.

if __name__=='__main__':main()
