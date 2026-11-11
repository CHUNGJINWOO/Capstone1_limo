#!/bin/bash
HERE=$(cd "$(dirname "$0")" && pwd)
for SC in wall ctrl wall2; do
  case $SC in wall) A="-1.0 -2.05 0.0";; wall2) A="-0.3 -2.05 0.0";; ctrl) A="-1.1 -0.6 1.5708";; esac
  for EST in f4 b0 f1b f3b f5_improved; do
    echo "== $EST $SC"; bash $HERE/run_estimator_probe.sh $EST $SC $A
  done
done
echo CHAIN_DONE
