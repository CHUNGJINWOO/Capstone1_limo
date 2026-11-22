"""Test-record guard (2026-11-22): every run gets ONE new directory; nothing existing is ever overwritten.

Why: on 2026-11-22 a wrapper script re-ran a regression into an existing results directory. The harnesses refused to start
(their own OUT.mkdir(exist_ok=False)), but the wrapper's console redirects (`> phaseN.out`, outside the run directory) had already
truncated the original console logs. Those logs are lost; this tool does not recover them.

  claim  ROOT NAME [--input PATH ...]   create ROOT/NAME (ROOT must exist; refuses if ROOT/NAME exists) and write
                                       ROOT/NAME/MANIFEST.json: run_id, start time, git HEAD + status digest, sha256 of the
                                       fleet_control agent/server/tests sources and of every --input (Gate script, params,
                                       maps...), selected environment, host. Prints the directory.
  finish DIR --rc N                    write DIR/FINISH.json once (rc, end time, source digest again, changed: bool).
  check  DIR                           verify DIR's MANIFEST inputs/sources against the current files (exit 1 on mismatch).

Wrappers put the harness OUT at DIR/run (a path that does not exist yet) and the console at DIR/console.out, using
`set -o noclobber`, so a repeated name fails before any file is touched.
"""
import argparse, datetime, hashlib, json, os, platform, subprocess, sys, uuid
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SRC_DIRS = ('fleet_control/agent', 'fleet_control/server', 'fleet_control/tests')
SRC_EXT = ('.py', '.js', '.cjs', '.sh', '.html', '.yaml', '.launch.py')
ENV_KEYS = ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION', 'GATE_SCRIPT', 'GATE_WRITER_STATUS',
            'GATE_INPUT_EXCL', 'GATE_INPUT_MAX_VX', 'GATE_INPUT_MAX_WZ', 'GATE_RESET_PEER', 'GATE_REQ_WD', 'NAV2_PARAMS_OVERLAY',
            'GZWEB_TIMELINE', 'GZWEB_HOME', 'GZ_SPAWN', 'GZ_WORLD', 'USE_SIM_TIME', 'ARBITER', 'OVERLAY', 'CAPSTONE45_SNAPSHOT')


def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()


def sources():
    out = {}
    for d in SRC_DIRS:
        for p in sorted((REPO / d).rglob('*')):
            if p.is_file() and p.name.endswith(SRC_EXT) and '__pycache__' not in p.parts and 'node_modules' not in p.parts:
                out[str(p.relative_to(REPO))] = sha(p)
    return out


def digest(d):
    return hashlib.sha256(json.dumps(d, sort_keys=True).encode()).hexdigest()


def git(*a):
    try: return subprocess.run(['git', '-C', str(REPO), *a], capture_output=True, text=True, timeout=60).stdout
    except Exception as e: return f'ERROR {e}'


def claim(a):
    root = Path(a.root).resolve()
    if not root.is_dir(): sys.exit(f'run_guard: ROOT {root} does not exist (create it deliberately first)')
    if '/' in a.name or a.name in ('', '.', '..'): sys.exit('run_guard: NAME must be a single path component')
    d = root / a.name
    try: d.mkdir()                                        # atomic, never parents=True / exist_ok=True
    except FileExistsError: sys.exit(f'run_guard: REFUSED, {d} already exists (records are never overwritten)')
    inputs = {}
    for p in a.input or []:
        q = Path(p).resolve(); inputs[str(q)] = sha(q) if q.is_file() else 'MISSING'
    src = sources()
    m = {'run_id': f'{a.name}_{uuid.uuid4().hex[:8]}', 'name': a.name, 'started': datetime.datetime.now().astimezone().isoformat(),
         'git_head': git('rev-parse', 'HEAD').strip(), 'git_status_sha256': hashlib.sha256(git('status', '--porcelain').encode()).hexdigest(),
         'sources_sha256': digest(src), 'sources': src, 'inputs': inputs,
         'env': {k: os.environ[k] for k in ENV_KEYS if k in os.environ},
         'host': {'python': platform.python_version(), 'kernel': platform.release(), 'machine': platform.machine()},
         'note': a.note or ''}
    with open(d / 'MANIFEST.json', 'x') as f: json.dump(m, f, indent=1)
    print(d)


def finish(a):
    d = Path(a.dir); m = json.loads((d / 'MANIFEST.json').read_text()); src = sources()
    changed = sorted(k for k in set(src) | set(m['sources']) if src.get(k) != m['sources'].get(k))
    doc = {'rc': a.rc, 'ended': datetime.datetime.now().astimezone().isoformat(), 'sources_sha256_end': digest(src),
           'sources_changed_during_run': changed}
    with open(d / 'FINISH.json', 'x') as f: json.dump(doc, f, indent=1)
    print(json.dumps({'rc': a.rc, 'sources_changed_during_run': len(changed)}))


def check(a):
    m = json.loads((Path(a.dir) / 'MANIFEST.json').read_text()); bad = []
    for p, h in m['inputs'].items():
        if (sha(p) if Path(p).is_file() else 'MISSING') != h: bad.append(p)
    src = sources(); bad += [k for k in m['sources'] if src.get(k) != m['sources'][k]]
    print(json.dumps({'mismatch': bad})); sys.exit(1 if bad else 0)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest='cmd', required=True)
    c = sp.add_parser('claim'); c.add_argument('root'); c.add_argument('name'); c.add_argument('--input', action='append'); c.add_argument('--note')
    f = sp.add_parser('finish'); f.add_argument('dir'); f.add_argument('--rc', type=int, required=True)
    k = sp.add_parser('check'); k.add_argument('dir')
    a = ap.parse_args(); {'claim': claim, 'finish': finish, 'check': check}[a.cmd](a)
