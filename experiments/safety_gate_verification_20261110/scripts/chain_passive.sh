#!/bin/bash
# Repeat the passive comparison (original explore drives; explore_relaxed dry-run observes) until a run shows the original
# explore_lite stopping while explore_relaxed(cap 60) still sees candidates, or MAXRUNS is reached.
cd "$(dirname "$0")/.."
E=explore_relaxed_eval; mkdir -p $E
MAXRUNS=${1:-4}
for k in $(seq 2 $((MAXRUNS+1))); do
  bash scripts/run_variant_sim.sh passive Q$k 330 > $E/Q$k.out 2>&1
  python3 - "$E/runs/Q$k" > $E/Q$k.check <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1] + '/relaxed_passive.jsonl')]
c0 = [r for r in rows if r['who'] == 'cap0' and r['sim_t'] > 20]; c60 = {round(r['sim_t'], 1): r for r in rows if r['who'] == 'cap60'}
hit = [(r['sim_t'], r['frontiers_kept'], c60.get(round(r['sim_t'], 1), {}).get('frontiers_kept')) for r in c0
       if r['frontiers_kept'] == 0 and (c60.get(round(r['sim_t'], 1), {}).get('frontiers_kept') or 0) > 0]
print('HIT' if hit else 'NOHIT', hit[:3])
PY
  if grep -q "^HIT" $E/Q$k.check; then break; fi
done
echo DONE > $E/passive_chain.done
