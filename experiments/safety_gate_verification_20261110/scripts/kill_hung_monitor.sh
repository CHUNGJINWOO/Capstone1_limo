#!/bin/bash
# Stop a hung monitor (sim clock stalled so its sim-time timers never fire) and clean the simulation.
pkill -f "[a]uto_mapping_monitor.py"
bash "$(dirname "$0")/cleanup_sim.sh"
