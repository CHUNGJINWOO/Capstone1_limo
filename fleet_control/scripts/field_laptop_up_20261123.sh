#!/bin/bash
# LAPTOP: start the Fleet Server (web console + API) for one field session (G2, 2026-11-23). One command, no SSH.
#   usage: field_laptop_up_20261123.sh SESSION_DIR PORT ROBOT_ID [ROBOT_ID...]
#   SESSION_DIR  a NEW directory for this session (fleet.db, map registry, tokens). It is created and initialised if it does not exist;
#                an existing one is reused as is (restart after a crash keeps the missions, maps and Lease history).
#   PORT         explicit, no default: the operational port 18190 is only used on purpose.
#   env NO_TAILSCALE=1  bind 127.0.0.1 only (rehearsal). Default: 127.0.0.1 + this laptop's Tailscale address (what the LIMO connects to).
#   env VIDEO_ORIGIN, ALLOW_FIELD_DATA: passed through to run_field_server.sh.
# Tokens are written to SESSION_DIR/secrets/*.token (0600). Give agent-ROBOT.token to the robot once (see FIELD_READINESS_20261123.md);
# paste operator.token into the web console. This script never prints a token and never starts anything on the robot.
set -euo pipefail
[ $# -ge 3 ] || { echo "usage: $0 SESSION_DIR PORT ROBOT_ID [ROBOT_ID...]" >&2; exit 2; }
SESSION=$1; PORT=$2; shift 2
[[ $PORT =~ ^[0-9]{4,5}$ ]] || { echo "ABORT: PORT must be 4-5 digits" >&2; exit 2; }
for r in "$@"; do [[ $r =~ ^[A-Za-z0-9_.-]{1,64}$ ]] || { echo "ABORT: bad robot id: $r" >&2; exit 2; }; done
FC=$(cd "$(dirname "$0")/.." && pwd)
umask 077
if [ ! -f "$SESSION/data/auth.json" ]; then
  mkdir -p "$SESSION"
  (cd "$FC/server" && PYTHONPATH="$FC/server" python3 -m fleet_server init --data-dir "$SESSION/data" --secrets-dir "$SESSION/secrets" --robots "$@" > "$SESSION/init.log")
  echo "initialised $SESSION (tokens in $SESSION/secrets, mode 0600)"
else
  echo "reusing $SESSION"
fi
for r in "$@"; do [ -f "$SESSION/secrets/agent-$r.token" ] || echo "WARN: no agent token for robot '$r' in $SESSION/secrets (was the session initialised with this robot?)" >&2; done
echo "Console: http://127.0.0.1:$PORT/   (Ctrl+C stops the server; the robot goes offline, nothing keeps moving: the Agent's stop and the Watchdog are local)"
exec "$FC/scripts/run_field_server.sh" "$SESSION/data" "$PORT"
