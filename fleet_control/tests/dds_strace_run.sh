#!/bin/bash
# Run ON the robot. usage: dds_strace_run.sh ID   (traces only our own new probe; no root; the stack is not touched)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
id=$1; X=/tmp/st_$id.xml
cat > $X <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="f" is_default_profile="true"><rtps><participantID>$id</participantID></rtps></participant></profiles>
XML
FASTRTPS_DEFAULT_PROFILES_FILE=$X strace -f -xx -s 400 -e trace=recvfrom -o /tmp/st_$id.trace python3 /tmp/dds_probe2.py "ID$id" --duration 8 --real /scan,/tf > /tmp/st_$id.json 2>/dev/null
OWN=$(python3 -c "import json;print(json.load(open('/tmp/st_$id.json')).get('own_participant_prefix'))")
echo "== ID $id: messages $(python3 -c "import json;d=json.load(open('/tmp/st_$id.json'));print({t:s['count'] for t,s in d['topics'].items()})") own_prefix=$OWN"
python3 /tmp/dds_strace_rtps.py /tmp/st_$id.trace $OWN
rm -f $X /tmp/st_$id.trace /tmp/st_$id.json
