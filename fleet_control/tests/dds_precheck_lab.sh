#!/bin/bash
# Isolated lab (own ROS domain, localhost only, nothing from the LIMO): false-positive / false-negative checks of scripts/dds_slot_precheck.py.
# usage: dds_precheck_lab.sh DOMAIN [REPS]
DOM=${1:?domain}; REPS=${2:-10}
HERE=$(cd "$(dirname "$0")" && pwd); PC=$HERE/../scripts/dds_slot_precheck.py
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
W=$(mktemp -d)
cat > $W/pub.py <<'PY'
import rclpy, sys, time
from std_msgs.msg import String
rate, life = float(sys.argv[1]), float(sys.argv[2]); names = sys.argv[3:]
rclpy.init(); n = rclpy.create_node('lab_pub'); ps = [n.create_publisher(String, t, 10) for t in names]
t0 = time.time(); nxt = t0
while life <= 0 or time.time() - t0 < life:
    import os
    if time.time() >= nxt and not os.path.exists(sys.argv[0] + '.stop'):
        for p in ps: p.publish(String(data='x'))
        nxt += 1.0 / rate
    rclpy.spin_once(n, timeout_sec=0.01)
rclpy.shutdown()
PY
T="/lab_a:std_msgs/msg/String,/lab_b:std_msgs/msg/String"
run(){ python3 $PC --topics "$1" --first-s ${2:-6} --fresh-s ${3:-2} --alternatives 1 2>/dev/null | python3 -c "import sys,json;d=json.loads(sys.stdin.read());print(d['verdict'], 'checked', [(c['id'],c['healthy']) for c in d['checked']])"; }
echo "== L1 healthy 20 Hz publisher x$REPS (false-positive rate)"
python3 $W/pub.py 20 0 /lab_a /lab_b >/dev/null 2>&1 & P=$!; sleep 2
ok=0; for i in $(seq 1 $REPS); do r=$(run "$T"); echo "  $i: $r"; [[ $r == SLOT_OK* ]] && ok=$((ok+1)); done; echo "  SLOT_OK $ok/$REPS"
echo "== L1b healthy 2 Hz publisher x3 (slow-topic sensitivity, fresh window 2 s)"
kill $P; wait $P 2>/dev/null; python3 $W/pub.py 2 0 /lab_a /lab_b >/dev/null 2>&1 & P=$!; sleep 2
for i in 1 2 3; do echo "  $i: $(run "$T")"; done
echo "== L1c healthy 0.5 Hz publisher x3 (expected weakness: window shorter than the period)"
kill $P; wait $P 2>/dev/null; python3 $W/pub.py 0.5 0 /lab_a /lab_b >/dev/null 2>&1 & P=$!; sleep 3
for i in 1 2 3; do echo "  $i: $(run "$T" 8 2)"; done
echo "   same 0.5 Hz publisher with --fresh-s 5: $(run "$T" 8 5)"
kill $P; wait $P 2>/dev/null
echo "== L2 required topic never published (publisher of /lab_a only)"
python3 $W/pub.py 20 0 /lab_a >/dev/null 2>&1 & P=$!; sleep 2; echo "  $(run "$T" 4 2)"; kill $P; wait $P 2>/dev/null
echo "== L3 publisher goes silent while the canary measures freshness (stale data must not pass), x3"
for i in 1 2 3; do rm -f $W/pub.py.stop; python3 $W/pub.py 20 0 /lab_a /lab_b >/dev/null 2>&1 & P=$!; sleep 1.5
  ( sleep 2.6; touch $W/pub.py.stop ) & python3 $PC --topics "$T" --first-s 6 --fresh-s 4 --alternatives 0 2>/dev/null | python3 -c "import sys,json;d=json.loads(sys.stdin.read());print('  ',d['verdict'],[(c['id'],c['healthy']) for c in d['checked']])"; wait %2 2>/dev/null; kill $P; wait $P 2>/dev/null; done
echo "== L4 no publisher at all"
echo "  $(run "$T" 4 2)"
rm -rf $W
