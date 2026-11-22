#!/bin/bash
# Run ON the robot. usage: dds_forced_id.sh ID [ID ...]  -> for each ID: start a read-only probe whose DDS participantID is forced
# via a temporary FastDDS profile (only for that probe process), report the ports it really got and the messages it received.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
for id in "$@"; do
  X=$(mktemp /tmp/fdds_XXXX.xml)
  cat > $X <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="forced" is_default_profile="true"><rtps><participantID>$id</participantID></rtps></participant>
</profiles>
XML
  FASTRTPS_DEFAULT_PROFILES_FILE=$X python3 /tmp/dds_probe.py forced$id --duration 9 > /tmp/fid.out 2>&1 & P=$!
  sleep 5
  ports=$(ss -ulnpH 2>/dev/null | grep "pid=$P," | awk '{print $4}' | sed 's/.*://' | grep -v '^26650$' | sort -n | tr '\n' ' ')
  wait $P
  echo "forced ID $id -> ports [$ports] $(python3 -c "
import json
r={}
for l in open('/tmp/fid.out'):
    try: d=json.loads(l); r[d['topic']]=d['count']
    except Exception: pass
print(' '.join('%s=%d'%(k,v) for k,v in r.items()) or 'NO OUTPUT')")"
  rm -f $X; sleep 2
done
