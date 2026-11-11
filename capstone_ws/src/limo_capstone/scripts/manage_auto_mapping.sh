#!/usr/bin/env bash
# manage_auto_mapping.sh - Convenient wrapper for LIMO Pro Auto-Mapping experiments
set -e

SOURCE_ENV="/home/pcu/load_experiment_env.sh"
if [ -f "$SOURCE_ENV" ]; then
    source "$SOURCE_ENV" >/dev/null 2>&1
fi

SCRIPT_PATH="/home/pcu/capstone_experiment_ws/src/limo_capstone/scripts/auto_mapping_experiment_manager.py"
if [ ! -f "$SCRIPT_PATH" ]; then
    SCRIPT_PATH="/home/pcu/auto_mapping_experiment_manager.py"
fi

python3 "$SCRIPT_PATH" "$@"
