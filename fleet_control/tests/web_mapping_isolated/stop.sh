#!/bin/bash
# stops only the process groups recorded by run.sh
RUN=$1
for p in $(tac $RUN/pgids); do kill -INT -- -$p 2>/dev/null; done; sleep 4
for p in $(cat $RUN/pgids); do kill -TERM -- -$p 2>/dev/null; done; sleep 2
for p in $(cat $RUN/pgids); do kill -KILL -- -$p 2>/dev/null; done; echo stopped
