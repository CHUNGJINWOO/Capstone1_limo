#!/bin/bash
# usage: run_timed.sh <model.sdf> : rotate cmd continuously, sample sim time + yaw ~every 1s
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
SHARE=$(ros2 pkg prefix limo_capstone)/share/limo_capstone; DESC=$(ros2 pkg prefix limo_description)/share
export IGN_GAZEBO_RESOURCE_PATH=$SHARE/models:$DESC:$SHARE/worlds GZ_SIM_RESOURCE_PATH=$SHARE/models:$DESC:$SHARE/worlds
ign gazebo -r -s $SHARE/worlds/lounge_evaluation_world.sdf -v 1 >/dev/null 2>&1 & GZ=$!
sleep 6; ros2 run ros_gz_sim create -world lounge_world -file "$1" -name limo -x -1.10 -y -0.60 -z 0.05 -Y 1.5708 >/dev/null 2>&1; sleep 4
( while true; do ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'angular: {z: 0.5}'; done ) & PUB=$!
for i in $(seq 1 8); do
  timeout 5 ign topic -e -t /world/lounge_world/pose/info -n 1 | python3 -c '
import sys,re,math
t=sys.stdin.read(); s=re.search(r"stamp \{\s*sec: (\d+)(?:\s*nsec: (\d+))?",t)
st=int(s.group(1))+int(s.group(2) or 0)*1e-9 if s else -1
for b in t.split("pose {")[1:]:
    if "name: \"limo\"" in b:
        o=re.search(r"orientation \{([^}]*)\}",b).group(1); g=lambda k:float((re.search(k+r": (\S+)",o) or [0,0])[1]) if re.search(k+r": (\S+)",o) else 0.0
        z,w=g("z"),g("w"); print("sim=%.2f yaw=%.2f"%(st,math.degrees(2*math.atan2(z,w if (w or z) else 1)))); break'
  sleep 0.7
done
kill $PUB; kill -9 $GZ; pkill -9 -f "[i]gn gazebo"; sleep 2
