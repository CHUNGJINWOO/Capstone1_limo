#!/bin/bash
cd "$(dirname "$0")/.."
bash scripts/chain_active.sh
bash scripts/chain_passive.sh 2
echo DONE > explore_relaxed_eval/all_chain.done
