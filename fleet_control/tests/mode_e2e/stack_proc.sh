#!/bin/bash
# stack_proc.sh RUN_DIR start|kill|stop|cont|stopall NAME [SIGNAL]
# Only process groups recorded in RUN_DIR/pgnames (started by this harness) are ever signalled.
RUN=$1; OP=$2; NAME=$3; SIG=${4:-KILL}
source $RUN/env.sh; source $RUN/cmds.sh
pgid() { awk -v n="$1" '$1==n{p=$2} END{print p}' $RUN/pgnames 2>/dev/null; }
case $OP in
  start)
    n=$(grep -c "^$NAME " $RUN/pgnames 2>/dev/null || true)
    eval "cmd=\$CMD_$NAME"
    setsid bash -c "$cmd" > $RUN/$NAME.$n.log 2>&1 < /dev/null &
    echo "$NAME $!" >> $RUN/pgnames; echo $! >> $RUN/pgids ;;
  kill)  p=$(pgid $NAME); [ -n "$p" ] && kill -$SIG -- -$p 2>/dev/null; true ;;
  stop)  p=$(pgid $NAME); [ -n "$p" ] && kill -STOP -- -$p 2>/dev/null; true ;;
  cont)  p=$(pgid $NAME); [ -n "$p" ] && kill -CONT -- -$p 2>/dev/null; true ;;
  stopall)
    for p in $(tac $RUN/pgids); do kill -INT -- -$p 2>/dev/null; done; sleep 4
    for p in $(tac $RUN/pgids); do kill -CONT -- -$p 2>/dev/null; kill -KILL -- -$p 2>/dev/null; done; true ;;
esac
