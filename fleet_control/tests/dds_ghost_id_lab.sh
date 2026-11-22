#!/bin/bash
# Isolated DDS lab (own ROS domain, localhost only, nothing from the robot or the LIMO stack): does reusing the participant ID of a
# SIGKILLed participant leave the next participant deaf, while the same reuse after a graceful exit works?
# usage: [VICTIM_PUBLISHES=1 VICTIM_AGE=25] dds_ghost_id_lab.sh DOMAIN [REPS]       needs ROS 2 Humble (rmw_fastrtps_cpp). Prints one line per case.
DOM=${1:?domain id, e.g. 92}; REPS=${2:-3}
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
W=$(mktemp -d)
xml(){ cat > $W/id$1.xml <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="f" is_default_profile="true"><rtps><participantID>$1</participantID></rtps></participant></profiles>
XML
}
cat > $W/pub.py <<'PY'
import rclpy, time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node('lab_pub'); p = n.create_publisher(String, '/lab', 10)
n.create_subscription(String, '/lab_v', lambda m: None, 10); n.create_subscription(String, '/lab_l', lambda m: None, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)); n.create_publisher(String, '/lab_pub_back', 10)
while True:
    p.publish(String(data='x')); rclpy.spin_once(n, timeout_sec=0.1)
PY
cat > $W/sub.py <<'PY'
import rclpy, sys, time
from std_msgs.msg import String
import os
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node('lab_sub_%d' % int(time.time()*1000 % 100000)); c = [0]
n.create_subscription(String, '/lab', lambda m: c.__setitem__(0, c[0] + 1), 10)
if os.environ.get('VICTIM_PUBLISHES'):   # long-lived victim that also owns reliable + transient-local writers, like the Agent/map nodes
    pubs = [n.create_publisher(String, '/lab_v', 10), n.create_publisher(String, '/lab_l', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    n.create_subscription(String, '/lab_pub_back', lambda m: None, 10)
end = time.time() + float(sys.argv[1])
while time.time() < end:
    for p in (pubs if os.environ.get('VICTIM_PUBLISHES') else []): p.publish(String(data='v'))
    rclpy.spin_once(n, timeout_sec=0.1)
print(c[0])
PY
run_sub(){ xml $1; FASTRTPS_DEFAULT_PROFILES_FILE=$W/id$1.xml python3 $W/sub.py $2 2>/dev/null; }
xml 0; FASTRTPS_DEFAULT_PROFILES_FILE=$W/id0.xml python3 $W/pub.py >/dev/null 2>&1 & PUB=$!
sleep 3
for r in $(seq 1 $REPS); do
  K=$((r*10+1)); G=$((r*10+2)); C=$((r*10+3))
  # SIGKILL victim at ID K
  xml $K; FASTRTPS_DEFAULT_PROFILES_FILE=$W/id$K.xml python3 $W/sub.py 90 >/dev/null 2>&1 & V=$!; sleep ${VICTIM_AGE:-4}; kill -9 $V; wait $V 2>/dev/null
  # graceful victim at ID G
  xml $G; FASTRTPS_DEFAULT_PROFILES_FILE=$W/id$G.xml python3 $W/sub.py 4 >/dev/null 2>&1; sleep 1
  echo "rep $r: reuse of SIGKILLed ID $K -> $(run_sub $K 6) msgs | reuse of gracefully-exited ID $G -> $(run_sub $G 6) msgs | never-used ID $C -> $(run_sub $C 6) msgs"
done
echo "later (ghost persistence): reuse of SIGKILLed ID 11 again after the run -> $(run_sub 11 6) msgs"
kill $PUB 2>/dev/null; wait $PUB 2>/dev/null; rm -rf $W
