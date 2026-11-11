#!/bin/bash
# Stage 1: processing_hz=20 only. Depth latency (same 300 s exploration conditions as stage 0) + short Nav2 goal per drive type.
cd "$(dirname "$0")/.."
D=sim_settings_20261111; mkdir -p $D
ACK=limo_integrated_auto_mapping_sim.launch.py; DIFF=limo_integrated_auto_mapping_sim_diff.launch.py
bash scripts/run_depth_latency.sh $ACK S1_ack_explore 300 > $D/S1_latency_ack.out 2>&1
bash scripts/run_depth_latency.sh $DIFF S1_diff_explore 300 > $D/S1_latency_diff.out 2>&1
bash scripts/run_nav_check.sh $ACK S1_ack_goal none > $D/S1_nav_ack.out 2>&1
bash scripts/run_nav_check.sh $DIFF S1_diff_goal none > $D/S1_nav_diff.out 2>&1
echo DONE > $D/stage1.done
