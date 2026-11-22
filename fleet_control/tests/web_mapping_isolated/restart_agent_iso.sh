#!/bin/bash
# Isolated runner helper: start a NEW Agent process for a run created by run_save.sh (same flags). usage: restart_agent_iso.sh RUN PORT DOMAIN  (OVERLAY env as for run_save.sh)
RUN=$1; PORT=$2; DOM=$3; HERE=$(cd "$(dirname "$0")" && pwd); FC=$(cd "$HERE/../.." && pwd); REPO=$(cd "$FC/.." && pwd)
source /opt/ros/humble/setup.bash; source $REPO/capstone_ws/install/setup.bash; source $OVERLAY/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$DOM ROS2CLI_NO_DAEMON=1 ROS_LOG_DIR=$RUN/roslog; unset ROS_DISCOVERY_SERVER
LINE=$(sed -n 22p $HERE/run_save.sh | sed -E 's/^start "(.*)" agent$/\1/'); REQ_WD=true
setsid bash -c "$(eval echo \"$(printf '%s' "$LINE" | sed 's/"/\\"/g')\")" > $RUN/agent2.log 2>&1 &
echo $! >> $RUN/pgids; echo "agent2 $!" >> $RUN/pgnames; echo "agent restarted pg=$!"
