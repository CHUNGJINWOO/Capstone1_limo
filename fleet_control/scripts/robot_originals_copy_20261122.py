#!/usr/bin/env python3
"""Protective copy of LIMO field originals to the laptop (2026-11-22). The robot is only READ.

usage: robot_originals_copy_20261122.py --out RUN_DIR --dest-root DIR --host USER@ROBOT_ADDRESS [--only NAME,...] [--dry-run]

Per source directory (all under the robot's /home/pcu):
  1. manifest: a read-only python helper is streamed to the robot over SSH (stdin, nothing is written there); it prints one JSON line
     per file (relative path, size, sha256). The manifest is stored in RUN_DIR/manifests/.
  2. reuse: if EVERY manifest file already exists on the laptop with the same sha256 (candidates found by name+size under --reuse-roots),
     the item is REUSED_VERIFIED and nothing is copied (the file mapping is written to RUN_DIR/reuse/).
  3. copy: rsync -a --partial (no --delete, no --remove-source-files, no overwrite of finished data) into DEST/.partial/<item>/ ; an
     interrupted run is resumed by running the tool again (the same .partial directory is continued).
  4. verify the partial copy: same file set (no missing, no extra), same sizes, same sha256, same file count and total bytes.
  5. rosbag checks (separate from the copy verdict): metadata.yaml present?, `ros2 bag info`, SQLite integrity_check and message count
     of every .db3 (opened read-only; a copy is used when a -wal file exists), message count compared with metadata.yaml.
  6. only if the copy verdict is PASS: rename .partial/<item> -> DEST/<item> (refused if it exists) and write DEST/<item>.COMPLETE.json.
Finished items are skipped on re-run. Verdicts: PASS / FAIL / SKIP / REUSED_VERIFIED. Bag integrity problems of an ORIGINAL are reported as
BAG_INTEGRITY_FAIL next to a PASS copy (the copy is faithful, the original is damaged).
"""
import argparse, hashlib, json, os, shutil, sqlite3, subprocess, sys, tempfile, time
from pathlib import Path

ROBOT_HOME = '/home/pcu'
ITEMS = []          # (remote parent relative to ROBOT_HOME, name)  -> local DEST/<group>/<name>
# 2026-11-24 LIMO cleanup: these were moved under ~/Capstone1_limo. The item KEY (and so the finished-item markers on the laptop) keeps the
# old logical parent; only the remote location changes. limo_locked_stack and the Agent state dirs were NOT moved.
RELOCATED_PARENTS = {'.local/state/limo-real-auto-mapping': 'Capstone1_limo/records/mapping/limo-real-auto-mapping'}
RELOCATED_ITEMS = {('.local/state', 'limo-depth-observe'): 'Capstone1_limo/records/validation/limo-depth-observe',
                   ('.local/state', 'limo_field_cam'): 'Capstone1_limo/records/video/limo_field_cam',
                   ('', 'maps_h107_20261120'): 'Capstone1_limo/records/maps/maps_h107_20261120'}


def remote_path(parent, name):
    if (parent, name) in RELOCATED_ITEMS: return f'{ROBOT_HOME}/{RELOCATED_ITEMS[(parent, name)]}'
    if parent in RELOCATED_PARENTS: return f'{ROBOT_HOME}/{RELOCATED_PARENTS[parent]}/{name}'
    return f'{ROBOT_HOME}/{parent}/{name}'.replace('//', '/')

HELPER = r'''
import hashlib, json, os, sys
root = sys.argv[1]
for dp, dn, fn in os.walk(root):
    dn.sort()
    for f in sorted(fn):
        p = os.path.join(dp, f)
        rel = os.path.relpath(p, root)
        if os.path.islink(p):
            print(json.dumps({'path': rel, 'link': os.readlink(p)})); continue
        h = hashlib.sha256(); n = 0
        with open(p, 'rb') as fh:
            while True:
                b = fh.read(1 << 20)
                if not b: break
                h.update(b); n += len(b)
        print(json.dumps({'path': rel, 'size': n, 'sha256': h.hexdigest()}), flush=True)
'''


def sh(*cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def ssh_ls(host, remote_dir):
    r = sh('ssh', '-o', 'BatchMode=yes', host, f'ls -1A {remote_dir}')
    if r.returncode:
        raise SystemExit(f'ssh ls failed: {r.stderr[:200]}')
    return [x for x in r.stdout.splitlines() if x]


def robot_manifest(host, remote_path):
    p = subprocess.Popen(['ssh', '-o', 'BatchMode=yes', host, 'python3', '-', remote_path], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    p.stdin.write(HELPER); p.stdin.close()
    rows = [json.loads(l) for l in p.stdout if l.strip()]
    if p.wait():
        raise RuntimeError('manifest failed')
    return rows


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        while True:
            b = fh.read(1 << 20)
            if not b: break
            h.update(b)
    return h.hexdigest()


def build_index(roots, skip):
    idx = {}
    for root in roots:
        for r, d, fs in os.walk(root):
            if any(r.startswith(s) for s in skip): d[:] = []; continue
            for f in fs:
                p = os.path.join(r, f)
                try: idx.setdefault((f, os.path.getsize(p)), []).append(p)
                except OSError: pass
    return idx


def try_reuse(manifest, idx):
    mapping = {}
    for m in manifest:
        if 'link' in m: return None
        found = None
        for cand in idx.get((os.path.basename(m['path']), m['size']), []):
            if sha256_file(cand) == m['sha256']: found = cand; break
        if m['size'] == 0 and found is None: found = '(empty file)'
        if found is None: return None
        mapping[m['path']] = found
    return mapping


def verify_copy(partial, manifest):
    exp = {m['path']: m for m in manifest if 'link' not in m}
    have = {}
    for r, d, fs in os.walk(partial):
        for f in fs:
            p = os.path.join(r, f); have[os.path.relpath(p, partial)] = p
    missing = sorted(set(exp) - set(have)); extra = sorted(set(have) - set(exp))
    bad = []
    for rel, m in exp.items():
        if rel in have:
            if os.path.getsize(have[rel]) != m['size'] or sha256_file(have[rel]) != m['sha256']: bad.append(rel)
    ok = not missing and not extra and not bad
    return ok, dict(files=len(exp), bytes=sum(m['size'] for m in exp.values()), missing=missing[:10], extra=extra[:10], mismatched=bad[:10])


def bag_checks(root):
    out = []
    bagdirs = sorted({str(Path(r)) for r, d, fs in os.walk(root) for f in fs if f.endswith('.db3')})
    for d in bagdirs:
        rec = dict(dir=os.path.relpath(d, root))
        meta = Path(d) / 'metadata.yaml'
        mc = None
        if meta.is_file():
            import re
            m = re.search(r'message_count:\s*(\d+)', meta.read_text()); mc = int(m.group(1)) if m else None
        rec['metadata_yaml'] = meta.is_file(); rec['metadata_message_count'] = mc
        total = 0; integ = []
        for db in sorted(Path(d).glob('*.db3')):
            tmp = None; src = str(db)
            try:
                if Path(str(db) + '-wal').exists():
                    tmp = tempfile.mkdtemp(); shutil.copy2(db, tmp); shutil.copy2(str(db) + '-wal', tmp)
                    if Path(str(db) + '-shm').exists(): shutil.copy2(str(db) + '-shm', tmp)
                    src = os.path.join(tmp, db.name); conn = sqlite3.connect(src)
                else:
                    conn = sqlite3.connect(f'file:{src}?mode=ro&immutable=1', uri=True)
                ic = conn.execute('PRAGMA integrity_check').fetchone()[0]
                cnt = conn.execute('SELECT count(*) FROM messages').fetchone()[0]
                conn.close(); total += cnt; integ.append({db.name: ic, 'messages': cnt})
            except Exception as e:
                integ.append({db.name: f'ERROR {type(e).__name__}: {str(e)[:80]}'})
            finally:
                if tmp: shutil.rmtree(tmp, ignore_errors=True)
        rec['sqlite'] = integ; rec['sqlite_messages'] = total
        if meta.is_file():
            r = sh('ros2', 'bag', 'info', d); rec['ros2_bag_info_rc'] = r.returncode
            import re
            m = re.search(r'Messages:\s*(\d+)', r.stdout); rec['ros2_bag_info_messages'] = int(m.group(1)) if m else None
        ok_int = all(list(i.values())[0] == 'ok' for i in integ)
        rec['verdict'] = 'PASS' if (ok_int and (mc is None or mc == total)) else 'BAG_INTEGRITY_FAIL'
        if not meta.is_file(): rec['note'] = 'no metadata.yaml: SQLite checks only'
        out.append(rec)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True); ap.add_argument('--dest-root', required=True)
    ap.add_argument('--host', required=True, help='USER@ADDRESS of the robot (no default: no network address is stored in this file)'); ap.add_argument('--only', default=''); ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--reuse-roots', default='local_data/raw_results,local_data/rosbags,experiments')
    a = ap.parse_args()
    out = Path(a.out); dest = Path(a.dest_root)
    for sub in ('manifests', 'reuse', 'results'): (out / sub).mkdir(parents=True, exist_ok=True)
    (dest / '.partial').mkdir(parents=True, exist_ok=True)
    items = []
    for n in ssh_ls(a.host, f'{ROBOT_HOME}/{RELOCATED_PARENTS[".local/state/limo-real-auto-mapping"]}'): items.append(('.local/state/limo-real-auto-mapping', n))
    for n in ssh_ls(a.host, f'{ROBOT_HOME}/.local/state/limo_locked_stack'): items.append(('.local/state/limo_locked_stack', n))
    for n in ('limo_robot_agent_field_20261120', 'limo_robot_agent_field', 'limo_robot_agent', 'limo-depth-observe', 'limo_field_cam'): items.append(('.local/state', n))
    items.append(('', 'maps_h107_20261120'))
    only = {x for x in a.only.split(',') if x}
    skip_roots = [str(dest), str(out)]
    idx = None; summary = []
    for parent, name in items:
        key = (parent.replace('.local/state/', '').replace('.local/state', 'state').replace('/', '__') or 'home') + '__' + name
        if only and name not in only and key not in only: continue
        remote = remote_path(parent, name)
        final = dest / key; partial = dest / '.partial' / key; marker = dest / f'{key}.COMPLETE.json'
        res = dict(item=key, remote=remote, started=time.strftime('%F %T'))
        if final.exists() and marker.exists():
            res['verdict'] = 'SKIP'; res['reason'] = 'already complete'; summary.append(res); print(json.dumps(res), flush=True); continue
        if final.exists():
            res['verdict'] = 'FAIL'; res['reason'] = 'destination exists without COMPLETE marker: not touched'; summary.append(res); print(json.dumps(res), flush=True); continue
        mf = robot_manifest(a.host, remote)
        (out / 'manifests' / f'{key}.robot.jsonl').write_text('\n'.join(json.dumps(m) for m in mf) + '\n')
        res['files'] = len([m for m in mf if 'link' not in m]); res['bytes'] = sum(m.get('size', 0) for m in mf)
        res['symlinks'] = [m for m in mf if 'link' in m]
        if a.dry_run:
            res['verdict'] = 'DRY_RUN'; summary.append(res); print(json.dumps(res), flush=True); continue
        if idx is None: idx = build_index([r for r in a.reuse_roots.split(',') if r], skip_roots)
        mp = try_reuse(mf, idx)
        if mp is not None and mf:
            (out / 'reuse' / f'{key}.json').write_text(json.dumps(mp, indent=1))
            res['verdict'] = 'REUSED_VERIFIED'; res['note'] = 'every file already on the laptop with identical sha256 (mapping in reuse/)'
            summary.append(res); print(json.dumps(res), flush=True); continue
        r = sh('rsync', '-a', '--partial', '--protect-args', '--compress', '--compress-choice=zstd', '--compress-level=3', '-e', 'ssh -o BatchMode=yes', f'{a.host}:{remote}/', f'{partial}/')   # link is ~2 MB/s and bag data compresses ~7x
        res['rsync_rc'] = r.returncode
        if r.returncode:
            res['verdict'] = 'FAIL'; res['reason'] = 'rsync: ' + r.stderr[:200]; summary.append(res); print(json.dumps(res), flush=True); continue
        ok, det = verify_copy(partial, mf); res['verify'] = det
        res['bags'] = bag_checks(partial)
        res['bag_verdict'] = 'BAG_INTEGRITY_FAIL' if any(b['verdict'] != 'PASS' for b in res['bags']) else ('PASS' if res['bags'] else 'NO_BAGS')
        if not ok:
            res['verdict'] = 'FAIL'; res['reason'] = 'copy verification failed (partial kept for inspection)'
        else:
            os.rename(partial, final)
            marker.write_text(json.dumps(dict(item=key, remote=remote, files=res['files'], bytes=res['bytes'], completed=time.strftime('%F %T'),
                                              manifest_sha256=hashlib.sha256((out / 'manifests' / f'{key}.robot.jsonl').read_bytes()).hexdigest(),
                                              bag_verdict=res['bag_verdict']), indent=1))
            res['verdict'] = 'PASS'
        res['finished'] = time.strftime('%F %T')
        (out / 'results' / f'{key}.json').write_text(json.dumps(res, indent=1)); summary.append(res)
        print(json.dumps({k: res[k] for k in ('item', 'verdict', 'files', 'bytes', 'bag_verdict') if k in res}), flush=True)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    cnt = {}
    for s in summary: cnt[s['verdict']] = cnt.get(s['verdict'], 0) + 1
    print('SUMMARY', cnt, flush=True)
    return 1 if cnt.get('FAIL') else 0


if __name__ == '__main__':
    sys.exit(main())
