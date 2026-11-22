#!/bin/bash
# Open-RMF (ROS 2 Humble) WITHOUT root and WITHOUT changing the system ROS packages: downloads the official .deb files of the Open-RMF Fleet Adapter and
# unpacks them into a private prefix, then writes an env.sh to source AFTER /opt/ros/humble/setup.bash.
# Tested with ros-humble-rmf-fleet-adapter-python 2.1.8 (25 packages, about 7.5 MB) on Ubuntu 22.04. Nothing is installed system-wide; the binaries are NOT part of this repository.
#
#   bash scripts/install_rmf_isolated.sh list       # show what would be downloaded (no download)
#   bash scripts/install_rmf_isolated.sh install    # download + unpack + write env.sh   (default)
#   bash scripts/install_rmf_isolated.sh env-only   # only (re)write env.sh for an existing prefix
# env: RMF_ISOLATED_DIR (default $HOME/rmf_isolated)   RMF_PACKAGE (default ros-humble-rmf-fleet-adapter-python)
set -euo pipefail
DIR=${RMF_ISOLATED_DIR:-$HOME/rmf_isolated}; PKG=${RMF_PACKAGE:-ros-humble-rmf-fleet-adapter-python}; MODE=${1:-install}
list() { apt-get -s install "$PKG" 2>/dev/null | awk '/^Inst/ {print $2}'; }
write_env() {
  cat > "$DIR/env.sh" <<ENVEOF
# isolated Open-RMF env (extracted debs; no system install). source AFTER /opt/ros/humble/setup.bash
P=$DIR/prefix
export AMENT_PREFIX_PATH=\$P/opt/ros/humble:\${AMENT_PREFIX_PATH:-}
export CMAKE_PREFIX_PATH=\$P/opt/ros/humble:\${CMAKE_PREFIX_PATH:-}
export LD_LIBRARY_PATH=\$P/opt/ros/humble/lib/x86_64-linux-gnu:\$P/usr/lib/x86_64-linux-gnu:\$P/opt/ros/humble/lib:\$P/opt/ros/humble/opt/rmf_traffic/lib:\${LD_LIBRARY_PATH:-}
export PYTHONPATH=\$P/opt/ros/humble/lib/python/site-packages:\$P/opt/ros/humble/local/lib/python3.10/dist-packages:\$P/opt/ros/humble/lib/python3.10/site-packages:\$P/usr/lib/python3/dist-packages:\${PYTHONPATH:-}
export PATH=\$P/opt/ros/humble/bin:\$PATH
export PYTHONNOUSERSITE=1   # Humble binaries need the system numpy 1.x; a user-site numpy 2.x segfaults rmf_adapter
ENVEOF
  echo "wrote $DIR/env.sh"
}
case "$MODE" in
  list)
    echo "candidate: $(apt-cache policy "$PKG" | awk '/Candidate/ {print $2}')  (tested: 2.1.8)"; list;;
  env-only)
    mkdir -p "$DIR/prefix"; write_env;;
  install)
    cand=$(apt-cache policy "$PKG" | awk '/Candidate/ {print $2}'); echo "candidate version: $cand (tested: 2.1.8)"
    case "$cand" in 2.1.*) ;; *) echo "WARNING: not a 2.1.x release; this adapter was only tested with 2.1.8";; esac
    mkdir -p "$DIR/debs" "$DIR/prefix"; pkgs=$(list); [ -n "$pkgs" ] || { echo "nothing to download (already installed system-wide?)"; }
    (cd "$DIR/debs" && apt-get download $pkgs)
    for f in "$DIR"/debs/*.deb; do dpkg -x "$f" "$DIR/prefix"; done
    write_env
    echo 'test: source /opt/ros/humble/setup.bash && source '"$DIR"'/env.sh && python3 -c "import rmf_adapter; print(\"rmf_adapter OK\")"';;
  *) echo "usage: $0 [list|install|env-only]"; exit 2;;
esac
