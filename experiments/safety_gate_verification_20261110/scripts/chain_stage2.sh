#!/bin/bash
# Stage 2: sim-only depth thresholds (min_height -0.02, costmap depth min_obstacle_height -0.03) on top of stage 1.
cd "$(dirname "$0")/.."
D=sim_settings_20261111; mkdir -p $D
ACK=limo_integrated_auto_mapping_sim.launch.py; DIFF=limo_integrated_auto_mapping_sim_diff.launch.py
for d in ack diff; do
  L=$ACK; [ $d = diff ] && L=$DIFF
  bash scripts/run_bench.sh $L S2_${d}_rail -1.2 0.5 3.14159 --rail-x 0.73 --rail-y0 -0.22 --rail-y1 0.12 --corridor 0.30 0.55 0.15 > $D/S2_${d}_rail.out 2>&1
  bash scripts/run_bench.sh $L S2_${d}_floor 3.5 -3.8 0.0 --corridor 0.40 1.50 0.40 > $D/S2_${d}_floor.out 2>&1
done
bash scripts/run_nav_check.sh $ACK S2_ack_goal none > $D/S2_nav_ack.out 2>&1
bash scripts/run_nav_check.sh $DIFF S2_diff_goal none > $D/S2_nav_diff.out 2>&1
bash scripts/run_nav_check.sh $ACK S2_ack_depthfault depth > $D/S2_navf_ack.out 2>&1
bash scripts/run_nav_check.sh $DIFF S2_diff_depthfault depth > $D/S2_navf_diff.out 2>&1
bash scripts/run_depth_latency.sh $ACK S2_ack_explore 300 > $D/S2_latency_ack.out 2>&1
bash scripts/run_depth_latency.sh $DIFF S2_diff_explore 300 > $D/S2_latency_diff.out 2>&1
echo DONE > $D/stage2.done
