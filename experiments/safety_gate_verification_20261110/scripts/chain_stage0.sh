#!/bin/bash
# Stage 0: measurements with the CURRENT (unchanged) repo launches = "before" values.
cd "$(dirname "$0")/.."
D=sim_settings_20261111; mkdir -p $D
ACK=limo_integrated_auto_mapping_sim.launch.py; DIFF=limo_integrated_auto_mapping_sim_diff.launch.py
bash scripts/run_depth_latency.sh $ACK S0_ack_explore 300 > $D/S0_latency_ack.out 2>&1
# rail bench (robot 0.73 m in front of the chair-3 sled rail) + free-floor corridor ahead of it
for d in ack diff; do
  L=$ACK; [ $d = diff ] && L=$DIFF
  bash scripts/run_bench.sh $L S0_${d}_rail -1.2 0.5 3.14159 --rail-x 0.73 --rail-y0 -0.22 --rail-y1 0.12 --corridor 0.30 0.55 0.15 > $D/S0_${d}_rail.out 2>&1
  bash scripts/run_bench.sh $L S0_${d}_floor 3.5 -3.8 0.0 --corridor 0.40 1.50 0.40 > $D/S0_${d}_floor.out 2>&1
done
echo DONE > $D/stage0.done
