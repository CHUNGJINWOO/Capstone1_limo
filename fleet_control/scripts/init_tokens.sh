#!/bin/bash
# One-time: create API tokens (hashes in data/auth.json, plain tokens in secrets/*.token, both mode 0600, both git-ignored).
cd "$(dirname "$0")/.."; PYTHONPATH=server python3 -m fleet_server init --data-dir data --secrets-dir secrets --robots pcu mock1 "$@"
