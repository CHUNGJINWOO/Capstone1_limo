#!/bin/bash
# Integrated simulation launcher (simulation only). usage: run_integrated.sh {up|down|status|all} [--run-dir DIR] [--scenarios a,b] [--repeats N]
# Required: CAPSTONE_REFERENCE_WS = built read-only reference workspace (install/setup.bash). Optional env: see README.md (ports, domains, run root).
set -u
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
: "${CAPSTONE_REFERENCE_WS:?set CAPSTONE_REFERENCE_WS to the built reference workspace}"
export CAPSTONE_REFERENCE_WS
set +u; source /opt/ros/humble/setup.bash; source "${RMF_ISOLATED_ENV:-$HOME/rmf_isolated/env.sh}"; set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${INT_RMF_DOMAIN:-182}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${TMPDIR:-/tmp}/limo_integrated_roslogs}"; mkdir -p "$ROS_LOG_DIR"
exec python3 -u "$HERE/run_integrated.py" "$@"
