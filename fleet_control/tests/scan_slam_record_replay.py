"""Synthetic isolated ROS recording and playback using the web Agent recorder."""
import argparse,importlib.util,json,os,pathlib,signal,subprocess,sys,time
root=pathlib.Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('session',root/'fleet_control/agent/limo_robot_agent/limo_robot_agent/recording_session.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):raise SystemExit('isolated ROS required')
d=pathlib.Path(a.output);d.mkdir(mode=0o700)
s=m.RecordingSession(d,'scan_slam',root/'capstone_ws/src/limo_capstone/scripts/run_recorder.py',root/'fleet_control/scripts/verify_bag_copy.py')
try:
 s.start()
 for _ in range(100):
  time.sleep(.1);st=s.status()
  if st['status']!='STARTING':break
 if st['status']!='RECORDING':raise RuntimeError(str(st))
 time.sleep(3)
 pub=subprocess.run(['ros2','topic','pub','--times','5','-r','2','/scan_slam','sensor_msgs/msg/LaserScan',"{header: {frame_id: 'laser'}, angle_min: 0.0, angle_max: 1.0, angle_increment: 1.0, range_min: 0.1, range_max: 10.0, ranges: [1.0, 2.0]}"],capture_output=True,text=True,timeout=25)
 (d/'publish.log').write_text(pub.stdout+pub.stderr)
 if pub.returncode:raise RuntimeError('publisher failed')
 time.sleep(2);st=s.stop(timeout=40)
 if st['status']!='VERIFIED':raise RuntimeError(str(st))
 bag=pathlib.Path(st['run_dir'])/'bag';info=subprocess.run(['ros2','bag','info',str(bag)],capture_output=True,text=True,timeout=30);(d/'bag-info.log').write_text(info.stdout+info.stderr)
 if info.returncode or 'Topic: /scan_slam' not in info.stdout:raise RuntimeError('scan_slam absent')
 with (d/'echo.log').open('w') as out:
  listener=subprocess.Popen(['ros2','topic','echo','--once','/scan_slam','sensor_msgs/msg/LaserScan','--qos-reliability','best_effort'],stdout=out,stderr=subprocess.STDOUT)
  time.sleep(3)
  play=subprocess.run(['ros2','bag','play',str(bag),'--delay','3'],capture_output=True,text=True,timeout=40)
  (d/'play.log').write_text(play.stdout+play.stderr)
  try:rc=listener.wait(timeout=15)
  except subprocess.TimeoutExpired:listener.send_signal(signal.SIGINT);listener.wait(timeout=8);raise RuntimeError('replay subscriber timed out')
 if play.returncode or rc or 'frame_id: laser' not in (d/'echo.log').read_text():raise RuntimeError('replay did not deliver scan')
 print(json.dumps({'result':'PASS','bag':str(bag),'output':str(d)}))
finally:
 if s.process and s.process.poll() is None and s.state['status'] in ('STARTING','RECORDING'):
  try:s.stop(timeout=40)
  except Exception:pass
 s.close()
