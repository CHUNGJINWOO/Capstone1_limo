#!/bin/bash
# usage: run_variant_sim.sh <passive|drive> <tag> <max_wall> [cap] [check(0|1)]
# Runs an explore_relaxed experiment launch (overlay workspace sourced) with the passive monitor and the frontier tracer.
# Never resets the gate, never publishes velocity itself.
HERE=$(cd "$(dirname "$0")" && pwd); EXP=$(dirname "$HERE")
MODE=$1; TAG=$2; MAXW=$3; CAP=${4:-0}; CHECK=${5:-0}
OUT=$EXP/explore_relaxed_eval/runs/$TAG; [ -e "$OUT" ] && { echo "refusing to overwrite $OUT"; exit 1; }; mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source ~/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash; source $EXP/variants/explore_variant_ws/install/setup.bash
export ROS_DOMAIN_ID=$((100 + $(date +%s) % 120)); export RELAXED_CAP=$CAP RELAXED_CHECK=$CHECK
cd ~/school/2026-2/Capstone1_limo/capstone_ws
setsid ros2 launch $EXP/variants/sim_diff_relaxed_$MODE.launch.py start_explore:=true > "$OUT/launch.log" 2>&1 &
sleep 1
if [ "$MODE" = passive ]; then
  python3 -u "$HERE/relaxed_listener.py" --out "$OUT" --seconds $MAXW > "$OUT/listener.log" 2>&1 &
  LPID=$!
fi
python3 -u "$HERE/auto_mapping_monitor.py" --out "$OUT" --launch-log "$OUT/launch.log" --max-wall $MAXW --tag $TAG > "$OUT/monitor.log" 2>&1
echo "monitor: $(tail -1 $OUT/monitor.log)"
[ -n "$LPID" ] && kill $LPID 2>/dev/null
bash "$HERE/cleanup_sim.sh"
