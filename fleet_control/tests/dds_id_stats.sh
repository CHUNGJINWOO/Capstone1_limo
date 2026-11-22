#!/bin/bash
# Run ON the robot (needs /tmp/dds_probe2.py). usage: dds_id_stats.sh DURATION "ID ID ..." [ENV=VAL ...]
# One read-only probe per ID (forced participantID); prints OK/FAIL by messages received on /scan and /tf.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
DUR=$1; IDS=$2; shift 2; for kv in "$@"; do export "$kv"; done
for id in $IDS; do
  X=/tmp/ids_$id.xml
  printf '<?xml version="1.0"?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="f" is_default_profile="true"><rtps><participantID>%s</participantID></rtps></participant></profiles>' $id > $X
  r=$(FASTRTPS_DEFAULT_PROFILES_FILE=$X python3 /tmp/dds_probe2.py s --duration $DUR --real /scan,/tf 2>/dev/null | python3 -c "
import sys,json
try:
    d=json.loads(sys.stdin.read().strip().splitlines()[-1]); t=d['topics']; a,b=t['/scan']['count'],t['/tf']['count']
    print('%s scan=%d tf=%d'%('OK' if a>0 and b>0 else 'FAIL',a,b))
except Exception as e: print('ERROR', e)")
  echo "ID $id $r"; rm -f $X; sleep 1
done
