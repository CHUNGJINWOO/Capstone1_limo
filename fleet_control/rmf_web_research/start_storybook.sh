#!/usr/bin/env bash
set -euo pipefail
SOURCE_DIR=${RMF_WEB_RESEARCH_ROOT:-/tmp/capstone-rmf-web-official-20261113}
[[ $(git -C "$SOURCE_DIR" rev-parse HEAD) == c1b13dc90c0f6983970df87f58004e011a322d2f ]] || { echo 'Source commit mismatch.' >&2; exit 1; }
python3 - <<'PY'
import socket
with socket.socket() as s:
 s.bind(('127.0.0.1',18410))
PY
cd "$SOURCE_DIR/packages/rmf-dashboard-framework"
export STORYBOOK_DISABLE_TELEMETRY=1 CI=1
exec node_modules/.bin/storybook dev --config-dir .storybook-capstone --host 127.0.0.1 --port 18410 --ci --no-open --disable-telemetry
