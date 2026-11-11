#!/bin/bash
# Active short runs: explore_relaxed is the explorer. A = cap 60 + Nav2 reachability, C = cap 0 control (original behaviour).
cd "$(dirname "$0")/.."
E=explore_relaxed_eval; mkdir -p $E
bash scripts/kill_stray_launches.sh > /dev/null 2>&1
bash scripts/run_variant_sim.sh drive A1_cap60_nav2 330 60 1 > $E/A1.out 2>&1
bash scripts/run_variant_sim.sh drive C1_cap0_control 330 0 0 > $E/C1.out 2>&1
bash scripts/run_variant_sim.sh drive A2_cap60_nav2 330 60 1 > $E/A2.out 2>&1
bash scripts/run_variant_sim.sh drive C2_cap0_control 330 0 0 > $E/C2.out 2>&1
echo DONE > $E/active_chain.done
