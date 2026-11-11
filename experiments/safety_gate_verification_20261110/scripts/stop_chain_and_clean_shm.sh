#!/bin/bash
# Stop the passive chain and any sim/monitor, then remove STALE FastDDS shared-memory files that were left behind by
# killed processes (owned by this user, only when no ROS process is running).
pkill -f "[c]hain_passive.sh"; pkill -f "[r]un_variant_sim.sh"; pkill -f "[a]uto_mapping_monitor.py"
bash "$(dirname "$0")/cleanup_sim.sh"
sleep 3
if [ "$(pgrep -fc '[r]os2|[r]clpy|/opt/ros')" -gt 1 ]; then echo "ROS processes still running; not touching /dev/shm"; pgrep -af '[r]os2|/opt/ros' | cut -c1-100; exit 1; fi
before=$(find /dev/shm -maxdepth 1 -user "$USER" -name 'fastrtps_*' | wc -l)
find /dev/shm -maxdepth 1 -user "$USER" -name 'fastrtps_*' -delete
echo "removed stale fastrtps shm files: $before"
