#!/bin/bash
# Run ON the robot. usage: dds_trials.sh MODE N [ENV=VAL ...]   MODE: alone | warm<SECONDS>   -> one summary line per trial.
# Read-only: subscribes only. The "warm" helper is a throw-away rclpy node (no topics).
MODE=$1; N=$2; shift 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
for kv in "$@"; do export "$kv"; done
for i in $(seq 1 $N); do
  H=""
  if [[ $MODE == warm* ]]; then
    python3 -c 'import rclpy,time
rclpy.init(); n=rclpy.create_node("dds_warm_helper"); t=time.time()
while time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)' >/dev/null 2>&1 & H=$!
    sleep ${MODE#warm}
  fi
  python3 /tmp/dds_probe.py "$MODE#$i" --duration 8 2>/dev/null | python3 -c "
import sys,json
r={}
for l in sys.stdin:
    d=json.loads(l); r[d['topic']]=d['count']
print(sys.argv[1], ' '.join('%s=%d'%(k,v) for k,v in r.items()))" "$MODE#$i $*"
  [ -n "$H" ] && { kill $H 2>/dev/null; wait $H 2>/dev/null; }
  sleep 2
done
