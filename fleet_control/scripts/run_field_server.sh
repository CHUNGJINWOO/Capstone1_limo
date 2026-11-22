#!/bin/bash
# Field control server (Mission Manager + web dashboard) from THIS repository's fleet_control/, foreground.
#
# usage: run_field_server.sh DATA_DIR [PORT]          PORT default 18190
#   DATA_DIR  fleet data dir (fleet.db, map_registry.db, map_store/, auth.json). Tokens = that dir's auth.json.
#   env NO_TAILSCALE=1   bind 127.0.0.1 only (isolated test). Default: 127.0.0.1 + this machine's Tailscale IPv4.
#   env VIDEO_ORIGIN=http://HOST:PORT   allow the dashboard video panel to connect to the AI host (CSP connect-src).
#   env ALLOW_FIELD_DATA=1   required to serve the operational field data dir (see FIELD_DATA below).
#
# Why this exists: the 2026-11-13 field launcher (experiments/real_limo_field_20261113/laptop/lf_t1_server.sh) reads
# FLEET_SRC from ~/.config/limo_field_20261113/kit.env, which points to the Capstone1_limo_video_preverify worktree
# released on 2026-11-14 after its code was merged into fleet_control/. That config is left untouched.
# Field profile: --require-supervisor-ready is always on (Codex 2026-11-21); a real-robot Agent also forces it server-side.
# Never binds 0.0.0.0. Ctrl+C stops it. Prints no tokens.
set -euo pipefail
DATA=${1:?usage: run_field_server.sh DATA_DIR [PORT]}; PORT=${2:-18190}
FC=$(cd "$(dirname "$0")/.." && pwd)
FIELD_DATA=$HOME/.local/share/robot-sim-projects/limo_field_20261113/data
DATA=$(cd "$DATA" && pwd)
[ -f "$DATA/auth.json" ] || { echo "ABORT: $DATA/auth.json missing (no tokens -> nobody could log in)"; exit 2; }
if [ "$DATA" = "$(cd "$FIELD_DATA" 2>/dev/null && pwd)" ] && [ "${ALLOW_FIELD_DATA:-0}" != 1 ]; then
  echo "ABORT: $DATA is the operational field data. Back it up first; set ALLOW_FIELD_DATA=1 only after approval."; exit 2
fi
if ss -ltn | grep -q ":$PORT "; then echo "ABORT: port $PORT already in use"; exit 3; fi
BIND=(--bind 127.0.0.1); [ "${NO_TAILSCALE:-0}" = 1 ] || BIND+=(--tailscale)
[ -z "${VIDEO_ORIGIN:-}" ] || BIND+=(--video-origin "$VIDEO_ORIGIN")
echo "fleet_server from $FC/server  data=$DATA  http://127.0.0.1:$PORT/  (Ctrl+C to stop)"
cd "$FC/server" && exec env PYTHONPATH="$FC/server" python3 -m fleet_server serve --data-dir "$DATA" --port "$PORT" "${BIND[@]}" --offline-after 6 \
  --require-supervisor-ready
