#!/bin/bash
# Quarantine the passive runs P2..P5 (their listeners outlived the runs and recorded messages from the NEXT run's simulation),
# make the run driver kill its listener, and restart the passive chain with fresh names (Q*).
cd "$(dirname "$0")/.."
E=explore_relaxed_eval
pkill -f "[r]elaxed_listener.py" 2>/dev/null
mkdir -p $E/_invalid_listener_crosstalk
for k in 2 3 4 5; do
  [ -e $E/runs/P$k ] && mv $E/runs/P$k $E/_invalid_listener_crosstalk/P$k
  mv $E/P$k.out $E/P$k.check $E/_invalid_listener_crosstalk/ 2>/dev/null
done
rm -f $E/passive_chain.done
python3 - <<'PY'
p = 'scripts/run_variant_sim.sh'; s = open(p).read()
if 'LPID' not in s:
    s = s.replace('''relaxed_listener.py" --out "$OUT" --seconds $MAXW > "$OUT/listener.log" 2>&1 &
fi''', '''relaxed_listener.py" --out "$OUT" --seconds $MAXW > "$OUT/listener.log" 2>&1 &
  LPID=$!
fi''')
    s = s.replace('echo "monitor: $(tail -1 $OUT/monitor.log)"', 'echo "monitor: $(tail -1 $OUT/monitor.log)"\n[ -n "$LPID" ] && kill $LPID 2>/dev/null')
    open(p, 'w').write(s)
c = open('scripts/cleanup_sim.sh').read()
if 'relaxed_listener' not in c:
    open('scripts/cleanup_sim.sh', 'w').write(c.replace('[f]rontier_tracer', '[f]rontier_tracer|[r]elaxed_listener'))
q = open('scripts/chain_passive.sh').read()
open('scripts/chain_passive.sh', 'w').write(q.replace('P$k', 'Q$k'))
PY
nohup bash scripts/chain_passive.sh 3 > /dev/null 2>&1 &
echo restarted
