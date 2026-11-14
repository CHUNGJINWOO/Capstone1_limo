#!/bin/bash
# Central server on the laptop. Binds ONLY 127.0.0.1 and the Tailscale address (never 0.0.0.0). Dashboard: http://<tailscale ip>:8080/ or http://127.0.0.1:8080/
cd "$(dirname "$0")/.."; exec env PYTHONPATH=server python3 -m fleet_server serve --data-dir data --port "${PORT:-8080}" --tailscale "$@"
