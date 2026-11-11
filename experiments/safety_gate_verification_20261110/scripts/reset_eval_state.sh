#!/bin/bash
# Kill every leftover sim/node and clear stale FastDDS shm. (The one-time quarantine step that used to live here was removed:
# run folder names were reused by the later valid, isolated runs.)
cd "$(dirname "$0")/.."
bash scripts/kill_stray_launches.sh
