#!/bin/bash
# usage: run_variant.sh <model.sdf>   -- spawns in lounge world, rotates 6s at wz=0.5, prints GT yaw delta
source /opt/ros/humble/setup.bash
source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash
SHARE=$(ros2 pkg prefix limo_capstone)/share/limo_capstone
DESC=$(ros2 pkg prefix limo_description)/share
WORLD=$SHARE/worlds/lounge_evaluation_world.sdf
export IGN_GAZEBO_RESOURCE_PATH=$SHARE/models:$DESC:$SHARE/worlds
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
DIR=$(dirname "$1")
ign gazebo -r -s "$WORLD" -v 1 > "$DIR/gz_$(basename "$1").log" 2>&1 &
GZ=$!
sleep 6
ros2 run ros_gz_sim create -world lounge_world -file "$1" -name limo -x -1.10 -y -0.60 -z 0.05 -Y 1.5708 >/dev/null 2>&1
sleep 4
pose() {
  timeout 5 ign topic -e -t /world/lounge_world/pose/info -n 1 | python3 -c '
import sys,re,math
t=sys.stdin.read()
for b in t.split("pose {")[1:]:
    if re.search(r"name: \"limo\"", b):
        def g(sec,k):
            m=re.search(sec+r" \{([^}]*)\}", b); s=m.group(1) if m else ""
            mm=re.search(k+r": (\S+)", s); return float(mm.group(1)) if mm else 0.0
        x,y=g("position","x"),g("position","y"); z,w=g("orientation","z"),g("orientation","w")
        if w==0 and z==0: w=1.0
        print("%.3f %.3f %.2f"%(x,y,math.degrees(2*math.atan2(z,w)))); break
'
}
P0=$(pose)
for i in $(seq 1 60); do ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'angular: {z: 0.5}'; sleep 0.1; done
ign topic -t /cmd_vel -m ignition.msgs.Twist -p 'angular: {z: 0.0}'
sleep 1
P1=$(pose)
echo "$(basename "$1"): before(x y yaw)=$P0  after=$P1"
kill -9 $GZ; pkill -9 -f "[i]gn gazebo"; sleep 2
