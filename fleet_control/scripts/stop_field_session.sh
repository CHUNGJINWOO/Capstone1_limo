#!/bin/bash
# Safe shutdown of the 2026-11-20 field session (motor-locked stack, Agent, video sender, AI host, 18190 server).
#
# usage: stop_field_session.sh            dry run: shows what is running and what WOULD be stopped (default)
#        stop_field_session.sh --execute  stops ONLY the processes started for this session, then verifies
#
# Robot side (over SSH, only these): the `ros2 launch limo_capstone ...` process group, `robot_agent`, `sender_node.py`.
# Laptop side: the AI host (pid file of today's run dir) and the 18190 server (pid taken from the 18190 listener).
# Nothing is deleted. Logs are copied to the laptop (kept under local_data/raw_results/limo_runs/<run_id>/).
# Never sends a motion command. If the robot is unreachable, only the laptop side is stopped and that is reported.
set -uo pipefail
EXEC=0; [ "${1:-}" = --execute ] && EXEC=1
REPO=$(cd "$(dirname "$0")/../.." && pwd)
RUN=$REPO/local_data/field_run_20261120_b1/run
OUT=$REPO/local_data/raw_results/limo_runs/20261120_shutdown_$(date +%H%M%S)
say() { echo "$*"; }
act() { if [ $EXEC = 1 ]; then "$@"; else say "  [dry-run] $*"; fi; }
IP=$(tailscale ip -4 pcu 2>/dev/null | head -1)
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 "pcu@$IP")
ROBOT_UP=0; [ -n "$IP" ] && "${SSH[@]}" true 2>/dev/null && ROBOT_UP=1

say "== robot ($([ $ROBOT_UP = 1 ] && echo reachable || echo UNREACHABLE))"
if [ $ROBOT_UP = 1 ]; then
  "${SSH[@]}" 'ps -eo pid,etime,args | grep -E "[s]ender_node|[r]obot_agent --ros|[r]os2 launch limo_capstone|[l]imo_base|[y]dlidar|[s]tall_detector" | cut -c1-100'
  if [ $EXEC = 1 ]; then
    "${SSH[@]}" 'pkill -INT -f "sender_node.py" ; pkill -INT -f "robot_agent --ros-args"; LP=$(pgrep -f "ros2 launch limo_capstone" | head -1); [ -n "$LP" ] && kill -INT -- -$(ps -o pgid= -p $LP | tr -d " "); for i in $(seq 25); do pgrep -f "ros2 launch limo_capstone|robot_agent --ros|sender_node.py" >/dev/null || break; sleep 1; done; echo "left: $(ps -eo args | grep -E "[s]ender_node|[r]obot_agent --ros|[r]os2 launch limo_capstone|[l]imo_base|[y]dlidar|[s]tall_detector|[c]omponent_container" | wc -l)"'
  else say "  [dry-run] SIGINT sender_node, robot_agent, launch group; wait up to 25 s; count leftovers"; fi
fi

say "== laptop"
AI_PID=$(cat "$RUN/ai_host.pid" 2>/dev/null); S_PID=$(ss -ltnp 2>/dev/null | grep ":18190 " | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
say "  ai_host pid=${AI_PID:-none} (alive: $(kill -0 "${AI_PID:-0}" 2>/dev/null && echo yes || echo no))  18190 server pid=${S_PID:-none}"
[ -n "${AI_PID:-}" ] && kill -0 "$AI_PID" 2>/dev/null && act kill -INT "$AI_PID"
[ -n "${S_PID:-}" ] && act kill -INT "$S_PID"
if [ $EXEC = 1 ]; then sleep 4; fi

if [ $EXEC = 1 ]; then
  mkdir -p "$OUT"
  if [ $ROBOT_UP = 1 ]; then
    rsync -a -e "ssh -o BatchMode=yes" "pcu@$IP:.local/state/limo_locked_stack/" "$OUT/locked_stack/" 2>/dev/null
    rsync -a -e "ssh -o BatchMode=yes" "pcu@$IP:.local/state/limo_robot_agent_field_20261120/agent.out" "$OUT/agent.out" 2>/dev/null
    rsync -a -e "ssh -o BatchMode=yes" "pcu@$IP:limo_video_kit/real_limo_kit/run/sender.log" "$OUT/sender.log" 2>/dev/null
    "${SSH[@]}" 'journalctl -k --since "08:40" 2>/dev/null | grep -i -E "usb|cp210"' > "$OUT/kernel_usb_events.txt" 2>/dev/null
  fi
  cp "$RUN"/ai_host.log "$OUT/" 2>/dev/null
  sed -i -E 's/100\.[0-9]+\.[0-9]+\.[0-9]+/<TS_IP>/g' "$OUT"/*.log "$OUT"/agent.out 2>/dev/null
  (cd "$OUT" && find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum > MANIFEST.sha256)
  say "== verify"
  ss -ltn | grep -E ":18190 |:1819[12] " | wc -l | sed 's/^/  laptop listeners left (expect 0): /'
  pgrep -f "ai_server.py|fleet_server serve" | wc -l | sed 's/^/  laptop processes left (expect 0): /'
  say "  logs saved: $OUT"
else
  say "== dry run only. Re-run with --execute to stop. Logs would be saved under $OUT"
fi
