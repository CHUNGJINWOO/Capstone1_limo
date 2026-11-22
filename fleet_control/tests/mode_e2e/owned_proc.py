"""Find a process the E2E harness itself started (never a machine-wide pgrep).

Only processes whose process group is recorded in RUN/pgnames (groups created by stack_proc.sh) or that descend from
such a group leader are candidates; exactly one must match the pattern, otherwise ValueError (no signal is sent).
"""
import re, subprocess
from pathlib import Path


def owned_pid(run_dir, pattern, groups=None):
    names = [l.split() for l in Path(run_dir, 'pgnames').read_text().splitlines() if l.strip()]
    leaders = {int(p) for n, p in names if groups is None or n in groups}
    rows = []
    for line in subprocess.run(['ps', '-eo', 'pid=,pgid=,ppid=,args='], capture_output=True, text=True).stdout.splitlines():
        parts = line.split(None, 3)
        if len(parts) == 4:
            rows.append((int(parts[0]), int(parts[1]), int(parts[2]), parts[3]))
    parent = {pid: ppid for pid, _, ppid, _ in rows}

    def owned(pid, pgid):
        if pgid in leaders:
            return True
        seen = 0
        while pid > 1 and seen < 64:
            if pid in leaders:
                return True
            pid = parent.get(pid, 1); seen += 1
        return False

    hits = [pid for pid, pgid, _, args in rows if re.search(pattern, args) and owned(pid, pgid) and 'owned_proc' not in args]
    if len(hits) != 1:
        raise ValueError(f'expected exactly one owned process for {pattern!r}, found {hits}')
    return hits[0]
