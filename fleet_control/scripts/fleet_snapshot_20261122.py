#!/usr/bin/env python3
"""File snapshot of the Fleet sources (2026-11-22). usage: fleet_snapshot_20261122.py DEST_DIR   (DEST_DIR must not exist; run from the repository root)
Writes tar.gz archives + SHA256 lists of: git-addable fleet_control files, ignored-but-useful evidence files, the common documents and the files other
parties modified (copies + `git diff --binary`). Token files and runtime data are listed by name/size/mode only (no content, no hash). Read-only toward git."""
import hashlib, json, os, subprocess, sys, tarfile, time
S = sys.argv[1]
if os.path.exists(S): sys.exit('refusing to write into an existing directory')
os.makedirs(S)
run = lambda *a: subprocess.run(a, capture_output=True, text=True).stdout
def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()
cand = [l for l in run('git', 'ls-files', '--others', '--exclude-standard', 'fleet_control').splitlines() if l]
ign = [l for l in run('git', 'ls-files', '--others', '--ignored', '--exclude-standard', 'fleet_control').splitlines() if l]
pyc = [f for f in ign if '__pycache__' in f]; other = [f for f in ign if '__pycache__' not in f]
sens = [f for f in other if f.startswith('fleet_control/secrets/') or f.startswith('fleet_control/data/')]
evid = [f for f in other if f not in sens]
common = [f for f in ('START_HERE.md', 'WORKTREES.md', 'VALIDATION.md', 'HANDOFF.md', 'CODE_INDEX.md', 'ARCHITECTURE.md', 'DECISIONS.md', 'CURRENT_STATUS.md') if os.path.exists(f)]
tracked_mod = [l[3:] for l in run('git', 'status', '--short').splitlines() if l.startswith(' M')]
owner_untracked = [f for f in ('capstone_ws/src/limo_capstone/config/integrated_nav2_depth_slow_params.yaml', 'capstone_ws/src/limo_capstone/scripts/start_locked_standby.py',
                               'capstone_ws/src/m-explore-ros2/explore/src/fleet_frontier_candidates.cpp') if os.path.exists(f)]
def tar(name, files):
    with tarfile.open(f'{S}/{name}', 'w:gz') as t:
        for f in sorted(files): t.add(f, arcname=f, recursive=False)
    open(f'{S}/{name}.sha256', 'w').write(f'{sha(f"{S}/{name}")}  {name}\n')
def lst(name, files):
    open(f'{S}/{name}', 'w').write(''.join(f'{sha(f)}  {f}\n' for f in sorted(files)))
tar('fleet_control_source.tar.gz', cand); lst('SHA256SUMS.source.txt', cand)
tar('fleet_control_ignored_evidence.tar.gz', evid); lst('SHA256SUMS.ignored_evidence.txt', evid)
tar('common_docs_and_owner_files.tar.gz', common + tracked_mod + owner_untracked); lst('SHA256SUMS.common_and_owner.txt', common + tracked_mod + owner_untracked)
open(f'{S}/owner_uncommitted_tracked_changes.patch', 'w').write(run('git', 'diff', '--binary'))
with open(f'{S}/SENSITIVE_NOT_COPIED.tsv', 'w') as o:
    o.write('path\tbytes\tmode\n'); [o.write(f'{f}\t{os.path.getsize(f)}\t{oct(os.stat(f).st_mode & 0o777)}\n') for f in sorted(sens)]
open(f'{S}/PYCACHE_EXCLUDED.txt', 'w').write(f'{len(pyc)} generated __pycache__ files excluded\n')
json.dump(dict(created=time.strftime('%F %T'), head=run('git', 'rev-parse', 'HEAD').strip(), branch=run('git', 'branch', '--show-current').strip(), candidates=len(cand),
               ignored_evidence=len(evid), sensitive_not_copied=len(sens), pycache=len(pyc), tracked_modified=tracked_mod, owner_untracked=owner_untracked), open(f'{S}/MANIFEST.json', 'w'), indent=1)
print(len(cand), len(evid), len(sens), len(pyc))
