#!/usr/bin/env bash
set -euo pipefail
RESEARCH_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SOURCE_DIR=${RMF_WEB_RESEARCH_ROOT:-/tmp/capstone-rmf-web-official-20261113}
COMMIT=c1b13dc90c0f6983970df87f58004e011a322d2f
if [[ ! -e "$SOURCE_DIR" ]]; then
  git clone --filter=blob:none --no-checkout https://github.com/open-rmf/rmf-web.git "$SOURCE_DIR"
  git -C "$SOURCE_DIR" checkout --detach "$COMMIT"
fi
[[ $(git -C "$SOURCE_DIR" rev-parse HEAD) == "$COMMIT" ]] || { echo 'Source commit differs; refusing changes.' >&2; exit 1; }
cd "$SOURCE_DIR"
# No system/ROS/Python install scripts. pnpm version recorded: 11.25.0.
HUSKY=0 CI=1 pnpm install --frozen-lockfile --ignore-scripts --filter rmf-dashboard-framework... --store-dir /tmp/capstone-rmf-web-pnpm-store
packages/api-client/node_modules/.bin/tsc --build packages/api-client/tsconfig.json
packages/rmf-models/node_modules/.bin/tsc --build packages/rmf-models/tsconfig.json
cp packages/rmf-models/package.json packages/rmf-models/dist/package.json
DEST=packages/rmf-dashboard-framework/.storybook-capstone
mkdir -p "$DEST/public/assets"
cp "$RESEARCH_DIR"/harness/{main.ts,preview.tsx,office-research.stories.tsx} "$DEST/"
cp packages/rmf-dashboard-framework/test-data/assets/office.png "$DEST/public/assets/office.png"
echo 'Prepared frontend only. Run start_storybook.sh separately.'
