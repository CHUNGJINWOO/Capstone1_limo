#!/bin/bash
# READ-ONLY inventory to run ON the LIMO (or through ssh). Writes nothing, changes nothing, starts no ROS node, touches no motor.
# usage (from the notebook, host/user supplied by the owner):  ssh <user>@<limo> 'bash -s' < limo_readonly_inventory.sh > limo_inventory.txt
for W in /home/pcu/capstone_ws /home/pcu/capstone_experiment_ws; do
  echo "##### $W"; [ -d "$W/src" ] || { echo "(missing)"; continue; }
  ls -la --time-style=long-iso "$W/src"
  for p in "$W"/src/*/; do
    n=$(find "$p" -type f -not -path '*/.git/*' -not -name '*.pyc' | wc -l); echo "pkg $(basename $p): files=$n git=$([ -d $p/.git ] && echo yes || echo no)"
  done
  echo "-- git state"; (cd "$W" && git status --short 2>&1 | head -40; git log -1 --format='%h %s' 2>&1)
  echo "-- install/ present: $([ -d $W/install ] && ls $W/install | tr '\n' ' ' || echo no)"
done
echo "##### file hashes (sha256, first 16) for the planned T1 set in BOTH workspaces (missing = not present)"
for W in /home/pcu/capstone_ws /home/pcu/capstone_experiment_ws; do
  echo "== $W"; (cd $W/src 2>/dev/null && for f in limo_capstone/package.xml limo_capstone/setup.py limo_capstone/limo_capstone/*.py limo_capstone/launch/*.py limo_capstone/config/* limo_depth_obstacles/limo_depth_obstacles/*.py; do [ -f "$f" ] && echo "$(sha256sum $f | cut -c1-16)  $f"; done)
done
echo "##### system"; . /etc/os-release; echo "$PRETTY_NAME"; uname -m; echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
ls /opt/ros 2>/dev/null; python3 -c "import numpy;print('numpy',numpy.__version__)" 2>&1; python3 -c "import scipy;print('scipy',scipy.__version__)" 2>&1 | tail -1
dpkg -l 2>/dev/null | awk '/ros-humble-(nav2|slam-toolbox|robot-localization|xacro|joint-state-publisher|tf2-ros|cv-bridge|image-transport|image-publisher|camera-info-manager|rmw-cyclonedds)/{print $2,$3}'
dpkg -l 2>/dev/null | awk '/libgoogle-glog|libopenni2|libeigen3|libboost-dev|libgeographic|libuvc|python3-colcon/{print $2,$3}'
ls /usr/local/lib 2>/dev/null | grep -i "ydlidar\|uvc"; ls /usr/local/include 2>/dev/null | grep -i ydlidar
echo "##### devices / udev"; ls -l /dev/ttylimo /dev/ydlidar /dev/video* 2>&1 | head; ls /etc/udev/rules.d | head -20
echo "##### disk/memory"; df -h /home | tail -1; free -h | sed -n 2p; nproc
echo "##### running ROS / motor-related processes (read-only)"; ps -eo pid,etime,cmd | grep -E "[r]os2|[l]imo_base|[t]eleop|[c]md_vel" | cut -c1-160
echo "##### shell env hooks"; ls -la /home/pcu/load_experiment_env.sh 2>&1; grep -n "ROS_DOMAIN_ID\|RMW_IMPLEMENTATION\|source" /home/pcu/.bashrc 2>/dev/null | tail -8
