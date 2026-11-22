#!/bin/bash
# Run ON the robot (read-only toward the stack: only new private participants + subscriptions to real topics).
# usage: dds_layers.sh   -> (1) real-topic layers for forced IDs 23/26, (2) outbound test between two NEW participants on private topics.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS2CLI_NO_DAEMON=1
xmlid(){ X=/tmp/lay_$1.xml; cat > $X <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="f" is_default_profile="true"><rtps><participantID>$1</participantID></rtps></participant></profiles>
XML
echo $X; }
sumline(){ python3 -c "
import sys,json
d=json.loads(sys.stdin.read().strip().splitlines()[-1])
print(d['label'], 'own_prefix=', d.get('own_participant_prefix'))
for t,s in d['topics'].items(): print('   %-10s pubs_discovered=%d graph_pubs=%d msgs=%d first_msg=%s prefixes=%s'%(t,s['pubs_discovered'],s['matched_max'],s['count'],s['t_first_msg'],[p[:24] for p in s['pub_prefixes']][:3]))
if d['ping']: print('   ping:', d['ping'])"; }
for id in 23 26; do FASTRTPS_DEFAULT_PROFILES_FILE=$(xmlid $id) python3 /tmp/dds_probe2.py "ID$id real" --duration 8 --real /scan,/tf,/map 2>/dev/null | sumline; done
echo "== outbound/inbound between two NEW participants (private topic /dds_diag/*; stack not involved)"
pair(){ a=$1; b=$2
  FASTRTPS_DEFAULT_PROFILES_FILE=$(xmlid $a) python3 /tmp/dds_probe2.py "A(ID$a)" --duration 9 --ping-pub A$a$b --ping-sub B$a$b > /tmp/pa.out 2>/dev/null &
  FASTRTPS_DEFAULT_PROFILES_FILE=$(xmlid $b) python3 /tmp/dds_probe2.py "B(ID$b)" --duration 9 --ping-pub B$a$b --ping-sub A$a$b > /tmp/pb.out 2>/dev/null &
  wait; echo "pair $a<->$b:"; sumline < /tmp/pa.out; sumline < /tmp/pb.out; }
pair 23 40; pair 40 23; pair 26 40; pair 23 26
rm -f /tmp/lay_*.xml /tmp/pa.out /tmp/pb.out
