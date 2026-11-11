#!/bin/bash
# usage: run_replay.sh <mode> <domain> <bag> <outdir> [rate]   (isolated ROS domain; slam_toolbox with the LIMO params; no hardware involved)
MODE=$1; DOM=$2; BAG=$3; OUT=$4; RATE=${5:-1.0}; EXTRA=${6:-}
S=$(dirname "$(readlink -f "$0")"); mkdir -p $OUT
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$DOM
setsid nohup ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=$S/bag2/config_backup/integrated_slam_params.yaml > $OUT/slam_$MODE.log 2>&1 < /dev/null &
SP=$!
sleep 10
python3 $S/replay_node.py $BAG --mode $MODE --rate $RATE $EXTRA --out $OUT/result_$MODE > $OUT/replay_$MODE.log 2>&1
pkill -INT -P $SP 2>/dev/null; pkill -INT -f "async_slam_toolbox_node.*ROS_DOMAIN_ID=$DOM" 2>/dev/null; sleep 1; kill -INT $SP 2>/dev/null; sleep 3
echo "done $MODE"
