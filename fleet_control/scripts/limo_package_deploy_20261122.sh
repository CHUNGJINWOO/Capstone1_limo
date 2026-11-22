#!/bin/bash
# Single-package deploy / rollback for a colcon workspace (2026-11-22, R3 hardened 2026-11-23).
# Reusable on the laptop (rehearsal) and on the LIMO (run over SSH with `bash -s`, so no script file is left on the robot).
# It never deletes: replaced directories/files are preserved in the backup directory. It starts no ROS node
# and touches no other package, no launch file, no driver, no serial port.
#
# usage: limo_package_deploy_20261122.sh ACTION [options...]
#   preflight     no robot_agent / limo_base / launch / bag process, workspace and package present, free disk >= 1 GB
#   backup        DIR/<PKG>_src_pre_STAMP.tar.gz + DIR/<PKG>_install_pre_STAMP.tar.gz, per-file source hash list, tar hashes
#   stage         verify TAR against TARSHA, extract into DIR/stage_STAMP (new), verify every file against FILES.sha256
#   replace       move WS/src/PKG to DIR/<PKG>_src_dir_replaced_STAMP, move DIR/stage_STAMP/PKG to WS/src/PKG
#   build         colcon build --packages-select PKG --symlink-install (ROS Humble + workspace overlay sourced)
#   verify        sha256sum -c FILES.sha256 from WS/src and import check (in --dry-run: read-only hash check only, no overlay source/import)
#   rollback      stage & verify backup archive first, then move WS/src/PKG to DIR/<PKG>_src_dir_failed_STAMP, swap, rebuild, verify
#   artifact      create deterministic reproducible tar + FILES.sha256 + TAR.sha256 + ARTIFACT.manifest.json from --src-dir (default --status candidate)
#   file-replace  safely backup, verify, atomically replace and post-verify a single regular file (--target --src --src-sha --backup-dir --stamp [--require-exec])
#   file-verify   verify a single regular file against --src-sha (--target --src-sha [--require-exec])
#   file-rollback stage & verify backup copy first, then atomically restore a single regular file (--target --backup-dir --stamp)
# Global option: --dry-run (read-only pre-checks and planned action output only; zero file/directory creation or overlay execution).
# Security: fail-closed rejection of symbolic links on targets and ancestor paths; strict package file allowlist & credential blocklist.
# Arguments hold no addresses or tokens. Exit codes: 0 ok, 3 abort (a safety condition failed, nothing further was done).
set -euo pipefail
abort() { echo "ABORT: $*" >&2; exit 3; }
ACTION=${1:?action}; shift
WS=; PKG=; DIR=; STAMP=; TAR=; TARSHA=; FILESSHA=
SRC_DIR=; OUT_DIR=; ART_STATUS=candidate; TARGET=; NEW_FILE=; SRC_SHA=; REQUIRE_EXEC=0; DRY_RUN=0
while [ $# -gt 0 ]; do case "$1" in
  --ws) WS=$2; shift 2;;
  --pkg) PKG=$2; shift 2;;
  --backup-dir) DIR=$2; shift 2;;
  --stamp) STAMP=$2; shift 2;;
  --tar) TAR=$2; shift 2;;
  --tar-sha) TARSHA=$2; shift 2;;
  --files-sha) FILESSHA=$2; shift 2;;
  --src-dir) SRC_DIR=$2; shift 2;;
  --out-dir) OUT_DIR=$2; shift 2;;
  --status) ART_STATUS=$2; shift 2;;
  --target) TARGET=$2; shift 2;;
  --src) NEW_FILE=$2; shift 2;;
  --src-sha) SRC_SHA=$2; shift 2;;
  --require-exec) REQUIRE_EXEC=1; shift 1;;
  --dry-run) DRY_RUN=1; shift 1;;
  *) abort "unknown argument $1";;
esac; done

no_processes() {
  local n
  n=$( { ps -eo args= | grep -E '(^|[ /])(robot_agent|limo_base|limo_base_node|stall_detector_node|motion_watchdog)( |$)|ros2 launch|ros2 bag|rosbag2' | grep -v -E 'grep|bash -c|bash -s|limo_package_deploy' || true; } | wc -l )
  [ "$n" -eq 0 ] || abort "$n robot/ROS process(es) running"
}

sha_of() { sha256sum "$1" | awk '{print $1}'; }
expected_sha() { awk 'NF>=1{print $1; exit}' "$1"; }

# Fail-closed path check: rejects '..', target symlinks, and any symlink ancestor component.
check_no_symlink_path() {
  local p="$1" label="${2:-path}" cur dir log_dir phys_dir
  [ -n "$p" ] || abort "$label is empty"
  case "$p" in
    */../*|*/..|../*|..) abort "$label must not contain '..' traversal components: $p" ;;
  esac
  [ ! -L "$p" ] || abort "$label must not be a symbolic link: $p"
  cur="$p"
  while [ -n "$cur" ] && [ "$cur" != "/" ] && [ "$cur" != "." ]; do
    [ ! -L "$cur" ] || abort "$label has a symbolic link ancestor ($cur): $p"
    cur=$(dirname "$cur")
  done
  dir=$(dirname "$p")
  if [ -d "$dir" ]; then
    log_dir=$(cd -L "$dir" 2>/dev/null && pwd -L) || abort "cannot resolve directory for $label: $dir"
    phys_dir=$(cd -P "$dir" 2>/dev/null && pwd -P) || abort "cannot resolve physical directory for $label: $dir"
    [ "$log_dir" = "$phys_dir" ] || abort "$label parent directory resolves through a symbolic link ($log_dir != $phys_dir)"
  fi
}

is_allowed_pkg_file() {
  local rel="$1"
  case "$rel" in
    package.xml|setup.py|setup.cfg|CMakeLists.txt|resource/*) return 0 ;;
    *.py|*.yaml|*.yml|*.md|*.txt|*.srv|*.msg|*.action|*.hpp|*.h|*.cpp|*.c|*.sh|*.service) return 0 ;;
    *.json)
      case "$rel" in
        *auth.json|*secret*|*token*|*credential*|*connection.json) return 1 ;;
        *) return 0 ;;
      esac
      ;;
    *) return 1 ;;
  esac
}

case "$ACTION" in
 artifact)
  [ -n "$SRC_DIR" ] && [ -n "$PKG" ] && [ -n "$OUT_DIR" ] && [ -n "$STAMP" ] || abort "--src-dir --pkg --out-dir --stamp are required"
  case "$PKG" in */*|"") abort "package name must be a plain directory name";; esac
  case "$ART_STATUS" in candidate|approved) ;; *) abort "--status must be 'candidate' or 'approved'";; esac
  check_no_symlink_path "$SRC_DIR" "src-dir"
  check_no_symlink_path "$OUT_DIR" "out-dir"
  [ -d "$SRC_DIR" ] || abort "source directory missing: $SRC_DIR"
  SYMLINKS=$(find "$SRC_DIR" -type l | head -n 5 || true)
  [ -z "$SYMLINKS" ] || abort "symbolic link not allowed in package source: $SYMLINKS"
  [ -f "$SRC_DIR/package.xml" ] || abort "required file missing: $SRC_DIR/package.xml"
  [ -f "$SRC_DIR/setup.py" ] || [ -f "$SRC_DIR/CMakeLists.txt" ] || abort "required build file missing (setup.py or CMakeLists.txt) in $SRC_DIR"
  if [ -f "$SRC_DIR/setup.py" ]; then
    [ -f "$SRC_DIR/$PKG/__init__.py" ] || abort "required module missing: $SRC_DIR/$PKG/__init__.py"
    if [ "$PKG" = "limo_robot_agent" ]; then
      for ep in robot_agent motion_watchdog mock_robot; do
        grep -q "$ep" "$SRC_DIR/setup.py" || abort "required entrypoint missing in setup.py: $ep"
      done
    fi
  fi
  SECRETS=$(find "$SRC_DIR" -type f \( -name '*.env' -o -name '.env*' -o -name '*.token' -o -name 'auth.json' -o -name 'real_connection.json' -o -name '*.key' -o -name '*.pem' -o -name '*.crt' -o -name '*.p12' -o -name '*.pfx' -o -name '*secret*' -o -name '*credential*' -o -name '*.db' -o -name '*.sqlite*' -o -name '*.db3' -o -name '*.bag' -o -name '*.mcap' -o -name '*.log' \) | head -n 5 || true)
  [ -z "$SECRETS" ] || abort "forbidden secret/credential/env/data file found in source: $SECRETS"
  TAR_NAME="${PKG}_${ART_STATUS}_${STAMP}.tar.gz"
  [ ! -e "$OUT_DIR/$TAR_NAME" ] && [ ! -e "$OUT_DIR/FILES.sha256" ] && [ ! -e "$OUT_DIR/TAR.sha256" ] || abort "artifact output already exists in $OUT_DIR (never overwritten)"
  FILE_LIST=$(cd "$SRC_DIR" && find . -type f ! -path '*/__pycache__/*' ! -name '*.pyc' ! -path '*/.git/*' | sed 's|^\./||' | LC_ALL=C sort)
  [ -n "$FILE_LIST" ] || abort "no package files found in $SRC_DIR"
  while IFS= read -r rel; do
    is_allowed_pkg_file "$rel" || abort "file not in artifact allowlist (forbidden or unclassified): $rel"
  done <<< "$FILE_LIST"
  FILE_COUNT=$(printf '%s\n' "$FILE_LIST" | wc -l)
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY-RUN artifact OK: pkg=$PKG status=$ART_STATUS (deployment_approved=false) stamp=$STAMP files=$FILE_COUNT out=$OUT_DIR/$TAR_NAME"
    exit 0
  fi
  mkdir -p "$OUT_DIR"
  TMP_STAGE=$(mktemp -d "${OUT_DIR}/.art_stage_${STAMP}.XXXXXX")
  trap 'rm -rf "$TMP_STAGE"' EXIT
  mkdir -p "$TMP_STAGE/$PKG"
  while IFS= read -r rel; do
    mkdir -p "$TMP_STAGE/$PKG/$(dirname "$rel")"
    cp -p "$SRC_DIR/$rel" "$TMP_STAGE/$PKG/$rel"
  done <<< "$FILE_LIST"
  (cd "$TMP_STAGE" && find "$PKG" -type f | LC_ALL=C sort | xargs sha256sum) > "$OUT_DIR/FILES.sha256"
  # Deterministic tar + gzip: sorted names, fixed UTC mtime, numeric uid/gid 0, gnu format (no PID/ctime pax headers), gzip -n (no timestamp/filename in gz header).
  (cd "$TMP_STAGE" && LC_ALL=C tar --sort=name --mtime='UTC 2020-01-01 00:00:00' --owner=0 --group=0 --numeric-owner --format=gnu -cf - "$PKG" | gzip -n -9 > "$OUT_DIR/$TAR_NAME")
  tar -tzf "$OUT_DIR/$TAR_NAME" >/dev/null || abort "created archive failed integrity listing"
  (cd "$OUT_DIR" && sha256sum "$TAR_NAME" > TAR.sha256)
  (cd "$OUT_DIR" && sha256sum -c --quiet TAR.sha256) || abort "TAR.sha256 self-check failed"
  GIT_HEAD=$(git -C "$SRC_DIR" rev-parse HEAD 2>/dev/null || echo "unknown")
  TAR_HASH=$(expected_sha "$OUT_DIR/TAR.sha256")
  FILES_HASH=$(sha_of "$OUT_DIR/FILES.sha256")
  python3 - "$OUT_DIR/ARTIFACT.manifest.json" "$PKG" "$ART_STATUS" "$STAMP" "$SRC_DIR" "$GIT_HEAD" "$FILE_COUNT" "$TAR_NAME" "$TAR_HASH" "$FILES_HASH" <<'PY'
import datetime, json, sys
out, pkg, status, stamp, src, head, count, tar_name, tar_sha, files_sha = sys.argv[1:]
manifest = {
    'pkg': pkg,
    'status': status,
    'deployment_approved': False,
    'approval_note': 'CLI metadata label only; does NOT constitute operator deployment approval or authorize robot connection',
    'stamp': stamp,
    'source_dir': src,
    'git_head': head,
    'file_count': int(count),
    'tar_file': tar_name,
    'tar_sha256': tar_sha,
    'files_sha256': files_sha,
    'created_at_utc': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
    'reproduce_cmd': f'limo_package_deploy_20261122.sh artifact --src-dir {src} --pkg {pkg} --out-dir <OUT_DIR> --stamp {stamp} --status {status}',
}
open(out, 'w').write(json.dumps(manifest, indent=2) + '\n')
PY
  echo "artifact OK: $OUT_DIR/$TAR_NAME ($FILE_COUNT files, status=$ART_STATUS [label only, deployment_approved=false], tar_sha256=${TAR_HASH:0:12}...)"
  exit 0;;

 file-verify)
  [ -n "$TARGET" ] && [ -n "$SRC_SHA" ] || abort "--target and --src-sha are required"
  check_no_symlink_path "$TARGET" "target"
  check_no_symlink_path "$SRC_SHA" "src-sha"
  [ -f "$TARGET" ] || abort "target file missing: $TARGET"
  [ -f "$SRC_SHA" ] || abort "checksum file missing: $SRC_SHA"
  EXP=$(expected_sha "$SRC_SHA"); ACT=$(sha_of "$TARGET")
  [ -n "$EXP" ] && [ "$EXP" = "$ACT" ] || abort "file checksum mismatch for $TARGET (expected=$EXP actual=$ACT)"
  if [ "$REQUIRE_EXEC" -eq 1 ] && [ ! -x "$TARGET" ]; then abort "target file is not executable: $TARGET"; fi
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN file-verify OK: $TARGET ($ACT)"; exit 0; fi
  echo "file-verify OK: $TARGET ($ACT)"
  exit 0;;

 file-replace)
  [ -n "$TARGET" ] && [ -n "$NEW_FILE" ] && [ -n "$SRC_SHA" ] && [ -n "$DIR" ] && [ -n "$STAMP" ] || abort "--target --src --src-sha --backup-dir --stamp are required"
  check_no_symlink_path "$TARGET" "target"
  check_no_symlink_path "$NEW_FILE" "src"
  check_no_symlink_path "$SRC_SHA" "src-sha"
  check_no_symlink_path "$DIR" "backup-dir"
  [ -f "$TARGET" ] || abort "target file missing: $TARGET"
  [ -r "$TARGET" ] || abort "target file not readable: $TARGET"
  TARGET_DIR=$(dirname "$TARGET")
  [ -w "$TARGET_DIR" ] || abort "target directory not writable: $TARGET_DIR"
  [ -f "$NEW_FILE" ] || abort "replacement source file missing: $NEW_FILE"
  [ -f "$SRC_SHA" ] || abort "checksum file missing: $SRC_SHA"
  EXP=$(expected_sha "$SRC_SHA"); NEW_ACT=$(sha_of "$NEW_FILE")
  [ -n "$EXP" ] && [ "$EXP" = "$NEW_ACT" ] || abort "new file checksum mismatch (expected=$EXP actual=$NEW_ACT)"
  if [ "$REQUIRE_EXEC" -eq 1 ] && [ ! -x "$NEW_FILE" ]; then abort "replacement file lacks execute permission: $NEW_FILE"; fi
  case "$NEW_FILE" in *.sh) bash -n "$NEW_FILE" || abort "bash syntax check failed: $NEW_FILE";; esac
  BASE=$(basename "$TARGET"); ORIG_MODE=$(stat -c '%a' "$TARGET"); ORIG_SHA=$(sha_of "$TARGET")
  BK="$DIR/${BASE}.pre_${STAMP}"; BK_SHA="$DIR/${BASE}.pre_${STAMP}.sha256"; BK_MODE="$DIR/${BASE}.pre_${STAMP}.mode"
  [ ! -e "$BK" ] && [ ! -e "$BK_SHA" ] && [ ! -e "$BK_MODE" ] || abort "backup already exists for stamp $STAMP (duplicate run refused): $BK"
  SAME_NOTE=""
  if [ "$ORIG_SHA" = "$NEW_ACT" ]; then SAME_NOTE=" (already identical to target)"; fi
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY-RUN file-replace OK: target=$TARGET mode=$ORIG_MODE backup=$BK new_sha=${NEW_ACT:0:12}...$SAME_NOTE"
    exit 0
  fi
  mkdir -p "$DIR"
  cp -p "$TARGET" "$BK" || abort "failed to create backup: $BK"
  printf '%s  %s\n' "$ORIG_SHA" "$BASE" > "$BK_SHA"
  printf '%s\n' "$ORIG_MODE" > "$BK_MODE"
  [ "$(sha_of "$BK")" = "$ORIG_SHA" ] || abort "backup verification failed: $BK"
  STAGED="$TARGET_DIR/.${BASE}.staging.${STAMP}.$$"
  if ! cp -p "$NEW_FILE" "$STAGED" || ! chmod "$ORIG_MODE" "$STAGED" || [ "$(sha_of "$STAGED")" != "$EXP" ]; then
    rm -f "$STAGED" 2>/dev/null || true
    abort "staging preparation or checksum verification failed before swap (target untouched)"
  fi
  check_no_symlink_path "$TARGET" "target"
  if ! mv -f "$STAGED" "$TARGET"; then
    rm -f "$STAGED" 2>/dev/null || true
    cp -p "$BK" "$TARGET" && chmod "$ORIG_MODE" "$TARGET" || abort "replace swap failed AND rollback failed!"
    [ "$(sha_of "$TARGET")" = "$ORIG_SHA" ] || abort "replace swap failed AND rollback checksum mismatch!"
    abort "replace failed during swap; rolled back cleanly to $BK"
  fi
  POST_SHA=$(sha_of "$TARGET")
  if [ "$POST_SHA" != "$EXP" ] || [ -L "$TARGET" ] || { [ "$REQUIRE_EXEC" -eq 1 ] && [ ! -x "$TARGET" ]; }; then
    cp -p "$BK" "$TARGET" && chmod "$ORIG_MODE" "$TARGET" || abort "post-verify failed AND rollback failed!"
    [ "$(sha_of "$TARGET")" = "$ORIG_SHA" ] || abort "post-verify failed AND rollback checksum mismatch!"
    abort "post-verify failed; rolled back cleanly to $BK"
  fi
  sync "$TARGET" "$TARGET_DIR" 2>/dev/null || true
  echo "file-replace OK: $TARGET updated (backup=$BK, sha=${POST_SHA:0:12}...)$SAME_NOTE"
  exit 0;;

 file-rollback)
  [ -n "$TARGET" ] && [ -n "$DIR" ] && [ -n "$STAMP" ] || abort "--target --backup-dir --stamp are required"
  check_no_symlink_path "$TARGET" "target"
  check_no_symlink_path "$DIR" "backup-dir"
  BASE=$(basename "$TARGET")
  TARGET_DIR=$(dirname "$TARGET")
  BK="$DIR/${BASE}.pre_${STAMP}"; BK_SHA="$DIR/${BASE}.pre_${STAMP}.sha256"; BK_MODE="$DIR/${BASE}.pre_${STAMP}.mode"
  [ -f "$BK" ] && [ ! -L "$BK" ] && [ -f "$BK_SHA" ] || abort "backup for stamp $STAMP not found: $BK"
  EXP=$(expected_sha "$BK_SHA"); BK_ACT=$(sha_of "$BK")
  [ -n "$EXP" ] && [ "$EXP" = "$BK_ACT" ] || abort "backup file corrupt (checksum mismatch): $BK"
  RESTORE_MODE="0600"
  if [ -f "$BK_MODE" ]; then
    RESTORE_MODE=$(cat "$BK_MODE")
    case "$RESTORE_MODE" in [0-7][0-7][0-7]|[0-7][0-7][0-7][0-7]) ;; *) abort "invalid backup mode in $BK_MODE: $RESTORE_MODE";; esac
  fi
  [ -d "$TARGET_DIR" ] && [ -w "$TARGET_DIR" ] || abort "target directory not writable for rollback: $TARGET_DIR"
  FAILED="$DIR/${BASE}.failed_${STAMP}"
  [ ! -e "$FAILED" ] || abort "failed-file archive already exists: $FAILED"
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY-RUN file-rollback OK: target=$TARGET from=$BK mode=$RESTORE_MODE sha=${BK_ACT:0:12}..."
    exit 0
  fi
  # Atomic rollback design: stage and verify the restored file in TARGET_DIR BEFORE moving or modifying TARGET.
  # If copy, chmod, or checksum verification fails, TARGET remains untouched and FAILED is not created (retryable).
  RB_STAGED="$TARGET_DIR/.${BASE}.rollback_staging.${STAMP}.$$"
  if ! cp -p "$BK" "$RB_STAGED" || ! chmod "$RESTORE_MODE" "$RB_STAGED"; then
    rm -f "$RB_STAGED" 2>/dev/null || true
    abort "rollback staging copy/chmod failed (existing target preserved untouched)"
  fi
  if [ ! -f "$RB_STAGED" ] || [ -L "$RB_STAGED" ] || [ "$(sha_of "$RB_STAGED")" != "$EXP" ]; then
    rm -f "$RB_STAGED" 2>/dev/null || true
    abort "rollback staged file checksum verification failed (existing target preserved untouched)"
  fi
  if [ -f "$TARGET" ]; then
    if ! (ln "$TARGET" "$FAILED" 2>/dev/null || cp -p "$TARGET" "$FAILED"); then
      rm -f "$RB_STAGED" "$FAILED" 2>/dev/null || true
      abort "failed to archive current target to $FAILED before swap (existing target preserved untouched)"
    fi
  fi
  check_no_symlink_path "$TARGET" "target"
  if ! mv -f "$RB_STAGED" "$TARGET"; then
    rm -f "$RB_STAGED" "$FAILED" 2>/dev/null || true
    abort "atomic rename of rollback staged file failed (existing target preserved)"
  fi
  [ "$(sha_of "$TARGET")" = "$EXP" ] || abort "restored file checksum mismatch after atomic rollback"
  sync "$TARGET" "$TARGET_DIR" 2>/dev/null || true
  echo "file-rollback OK: restored $TARGET from $BK (sha=${EXP:0:12}...)"
  exit 0;;
esac

[ -n "$WS" ] && [ -n "$PKG" ] && [ -n "$DIR" ] && [ -n "$STAMP" ] || abort "--ws --pkg --backup-dir --stamp are required"
case "$PKG" in */*|"") abort "package name must be a plain directory name";; esac
check_no_symlink_path "$WS" "ws"
check_no_symlink_path "$DIR" "backup-dir"
SRC="$WS/src/$PKG"; INST="$WS/install/$PKG"
check_no_symlink_path "$SRC" "package-src"
case "$ACTION" in
 preflight)
  no_processes; [ -d "$SRC" ] || abort "package source missing: $SRC"; [ -f "$SRC/package.xml" ] || abort "no package.xml in $SRC"
  FREE=$(df -Pk "$WS" | awk 'NR==2{print $4}'); [ "$FREE" -gt 1000000 ] || abort "less than 1 GB free"
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN preflight OK: no processes, $SRC present, $((FREE/1024)) MB free"; exit 0; fi
  echo "preflight OK: no processes, $SRC present, $((FREE/1024)) MB free";;
 backup)
  no_processes; S="$DIR/${PKG}_src_pre_$STAMP.tar.gz"; I="$DIR/${PKG}_install_pre_$STAMP.tar.gz"; L="$DIR/${PKG}_src_pre_$STAMP.files.sha256"
  for f in "$S" "$I" "$L"; do [ ! -e "$f" ] || abort "backup file exists (never overwritten): $f"; done
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN backup OK: src=$SRC -> $S"; exit 0; fi
  mkdir -p "$DIR"
  tar -czf "$S" -C "$WS/src" "$PKG"; [ -d "$INST" ] && tar -czf "$I" -C "$WS/install" "$PKG" || echo "(no install dir for $PKG: nothing to back up there)"
  (cd "$SRC" && find . -type f ! -path '*/__pycache__/*' ! -name '*.pyc' | LC_ALL=C sort | xargs sha256sum) > "$L"
  tar -tzf "$S" >/dev/null; (cd "$DIR" && sha256sum "$(basename "$S")" $( [ -f "$I" ] && basename "$I" ) > "${PKG}_pre_$STAMP.tars.sha256"); cat "$DIR/${PKG}_pre_$STAMP.tars.sha256"
  echo "backup OK: $(wc -l < "$L") source files listed";;
 stage)
  [ -f "$TAR" ] && [ -f "$TARSHA" ] && [ -f "$FILESSHA" ] || abort "--tar --tar-sha --files-sha must exist"
  check_no_symlink_path "$TAR" "tar"
  check_no_symlink_path "$TARSHA" "tar-sha"
  check_no_symlink_path "$FILESSHA" "files-sha"
  (cd "$(dirname "$TAR")" && sha256sum -c --quiet "$TARSHA") || abort "artifact tar hash mismatch"
  tar -tzf "$TAR" >/dev/null 2>&1 || abort "corrupt archive: $TAR"
  ST="$DIR/stage_$STAMP"; [ ! -e "$ST" ] || abort "stage directory exists: $ST"
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN stage OK: tar=$TAR -> $ST"; exit 0; fi
  mkdir -p "$ST"
  if ! tar -xzf "$TAR" -C "$ST" 2>/dev/null; then rm -rf "$ST"; abort "corrupt archive extraction failed: $TAR"; fi
  (cd "$ST" && sha256sum -c --quiet "$FILESSHA") || abort "extracted files differ from FILES.sha256"
  [ -d "$ST/$PKG" ] || abort "artifact does not contain $PKG/"
  echo "stage OK: $(find "$ST/$PKG" -type f | wc -l) files verified in $ST";;
 replace)
  no_processes; ST="$DIR/stage_$STAMP"; [ -d "$ST/$PKG" ] || abort "nothing staged"; OLD="$DIR/${PKG}_src_dir_replaced_$STAMP"; [ ! -e "$OLD" ] || abort "exists: $OLD"
  [ -f "$DIR/${PKG}_src_pre_$STAMP.tar.gz" ] || abort "no backup for this stamp: refusing to replace"
  [ -d "$SRC" ] || abort "target package source missing: $SRC"
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN replace OK: $SRC -> $OLD, $ST/$PKG -> $SRC"; exit 0; fi
  mv "$SRC" "$OLD"
  if ! mv "$ST/$PKG" "$SRC"; then
    if [ -d "$OLD" ] && [ ! -e "$SRC" ] && mv "$OLD" "$SRC" && [ -d "$SRC" ] && [ ! -L "$SRC" ]; then
      abort "replace move failed (staged -> $SRC); compensation verified: previous $SRC restored from $OLD"
    fi
    SRC_STATE="ABSENT"; [ -e "$SRC" ] && SRC_STATE="PRESENT_UNVERIFIED"
    OLD_STATE="ABSENT"; [ -d "$OLD" ] && OLD_STATE="PRESERVED"
    echo "STATE: SRC=$SRC ($SRC_STATE), OLD=$OLD ($OLD_STATE)" >&2
    echo "RECOVERY: restore previous package with 'mv \"$OLD\" \"$SRC\"'" >&2
    abort "replace move failed AND compensation failed: SRC is $SRC_STATE; previous package preserved at $OLD (not deleted)"
  fi
  echo "replace OK: old directory moved to $OLD";;
 build)
  no_processes
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN build OK: colcon build --packages-select $PKG --symlink-install in $WS (not executed)"; exit 0; fi
  set +u; source /opt/ros/humble/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
  (cd "$WS" && colcon build --packages-select "$PKG" --symlink-install 2>&1 | tail -6); echo "build finished";;
 verify)
  [ -f "$FILESSHA" ] || abort "--files-sha required"
  check_no_symlink_path "$FILESSHA" "files-sha"
  (cd "$WS/src" && sha256sum -c --quiet "$FILESSHA") || abort "source differs from FILES.sha256"
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "DRY-RUN verify OK: FILES.sha256 verified in $WS/src (overlay source and python import skipped in dry-run for zero mutation)"
    exit 0
  fi
  set +u; source /opt/ros/humble/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
  (cd "$WS" && PYTHONDONTWRITEBYTECODE=1 python3 -c "import $PKG, os; print('import OK from', os.path.dirname($PKG.__file__))"); echo "verify OK";;
 rollback)
  no_processes; S="$DIR/${PKG}_src_pre_$STAMP.tar.gz"; L="$DIR/${PKG}_src_pre_$STAMP.files.sha256"; [ -f "$S" ] && [ -f "$L" ] || abort "backup for stamp $STAMP not found"
  (cd "$DIR" && sha256sum -c --quiet "${PKG}_pre_$STAMP.tars.sha256") || abort "backup tar hash mismatch"
  tar -tzf "$S" >/dev/null 2>&1 || abort "backup archive corrupt: $S"
  FAILED="$DIR/${PKG}_src_dir_failed_$STAMP"; [ ! -e "$FAILED" ] || abort "exists: $FAILED"
  if [ "$DRY_RUN" -eq 1 ]; then echo "DRY-RUN rollback OK: $SRC -> $FAILED, restore from $S"; exit 0; fi
  # Stage and verify the backup extraction BEFORE moving current $SRC.
  # Note: swapping directories via two mv operations (SRC->FAILED, RB_STAGE/PKG->SRC) is NOT globally atomic;
  # compensation (FAILED->SRC) is explicitly attempted and verified on swap failure, and FAILED is never deleted.
  RB_STAGE="$WS/src/.rb_stage_${PKG}_${STAMP}.$$"
  mkdir -p "$RB_STAGE"
  if ! tar -xzf "$S" -C "$RB_STAGE" 2>/dev/null || [ ! -d "$RB_STAGE/$PKG" ] || ! (cd "$RB_STAGE/$PKG" && sha256sum -c --quiet "$L"); then
    rm -rf "$RB_STAGE" 2>/dev/null || true
    abort "rollback staging extraction or hash verification failed (existing $SRC preserved untouched)"
  fi
  HAD_PREV_SRC=0
  HAD_PREV_PKG_XML=0
  PREV_HASH_FILE="$RB_STAGE/.prev_src.sha256"
  if [ -d "$SRC" ]; then
    HAD_PREV_SRC=1
    [ -f "$SRC/package.xml" ] && HAD_PREV_PKG_XML=1
    (cd "$SRC" && find . -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 | LC_ALL=C sort -z | xargs -0 -r sha256sum) > "$PREV_HASH_FILE"
    if ! mv "$SRC" "$FAILED"; then
      rm -rf "$RB_STAGE" 2>/dev/null || true
      abort "failed to move current $SRC to $FAILED (existing $SRC preserved untouched)"
    fi
  fi
  if ! mv "$RB_STAGE/$PKG" "$SRC"; then
    if [ "$HAD_PREV_SRC" -eq 1 ]; then
      COMP_OK=0
      if [ -d "$FAILED" ] && [ ! -e "$SRC" ] && mv "$FAILED" "$SRC"; then
        if [ -d "$SRC" ] && [ ! -L "$SRC" ] && { [ "$HAD_PREV_PKG_XML" -eq 0 ] || [ -f "$SRC/package.xml" ]; } && { [ ! -s "$PREV_HASH_FILE" ] || (cd "$SRC" && sha256sum -c --quiet "$PREV_HASH_FILE"); }; then
          COMP_OK=1
        fi
      fi
      rm -rf "$RB_STAGE" 2>/dev/null || true
      if [ "$COMP_OK" -eq 1 ]; then
        abort "rollback swap failed (staged -> $SRC); compensation succeeded and verified previous $SRC is intact (rollback did NOT complete; backup $S preserved)"
      fi
      SRC_STATE="ABSENT"; [ -e "$SRC" ] && SRC_STATE="PRESENT_UNVERIFIED"
      FAILED_STATE="ABSENT"; [ -d "$FAILED" ] && FAILED_STATE="PRESERVED"
      echo "ERROR: rollback swap failed AND compensation move ($FAILED -> $SRC) failed or failed verification." >&2
      echo "STATE: SRC=$SRC ($SRC_STATE), FAILED=$FAILED ($FAILED_STATE), BACKUP_TAR=$S" >&2
      echo "RECOVERY: restore previous package with 'mv \"$FAILED\" \"$SRC\"' OR restore backup with 'tar -xzf \"$S\" -C \"$WS/src\" && (cd \"$SRC\" && sha256sum -c \"$L\")'" >&2
      abort "rollback failed and compensation failed: SRC is $SRC_STATE; previous package preserved at $FAILED (not deleted)"
    else
      rm -rf "$RB_STAGE" 2>/dev/null || true
      SRC_STATE="ABSENT"; [ -e "$SRC" ] && SRC_STATE="PRESENT_UNVERIFIED"
      echo "STATE: SRC=$SRC ($SRC_STATE), BACKUP_TAR=$S" >&2
      echo "RECOVERY: extract backup with 'tar -xzf \"$S\" -C \"$WS/src\" && (cd \"$SRC\" && sha256sum -c \"$L\")'" >&2
      abort "rollback swap failed (staged -> $SRC): SRC is $SRC_STATE; backup preserved at $S"
    fi
  fi
  rm -rf "$RB_STAGE" 2>/dev/null || true
  [ -d "$SRC" ] && [ ! -L "$SRC" ] && [ -f "$SRC/package.xml" ] || abort "restored package directory missing or invalid after rollback: $SRC"
  (cd "$SRC" && sha256sum -c --quiet "$L") || abort "restored source differs from the pre-deploy list"
  set +u; source /opt/ros/humble/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
  (cd "$WS" && colcon build --packages-select "$PKG" --symlink-install 2>&1 | tail -4)
  echo "rollback OK: restored $(wc -l < "$L") files (failed deploy kept in $FAILED)";;
 *) abort "unknown action $ACTION";;
esac
