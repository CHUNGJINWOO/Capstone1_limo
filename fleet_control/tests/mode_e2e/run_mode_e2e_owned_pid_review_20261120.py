"""Run the existing E2E with f06 limited to the stack process launched by this run.

No source files are changed. Its only substitution replaces machine-wide pgrep
with a check that exactly one bt_navigator is a descendant of our launch PID.
"""
from pathlib import Path

source_path=Path(__file__).with_name('run_mode_e2e.py')
source=source_path.read_text()
old="""    pid = subprocess.run(['pgrep', '-f', '__node:=bt_navigator|nav2_bt_navigator/bt_navigator'], capture_output=True, text=True).stdout.split()
    pid = [p for p in pid if p != str(os.getpid())]
    os.kill(int(pid[0]), 19)          # SIGSTOP: cancel and result are never answered
    t0 = time.monotonic(); request_mode('MANUAL')
    stopped = still_after(t0, .5, 1.0)
    rec, hist, secs = wait_mode('MANUAL', 45)
    status = mission_status(mid)
    os.kill(int(pid[0]), 18)          # SIGCONT
"""
new="""    import re
    stack_ids = [int(line.split()[1]) for line in (RUN / 'pgnames').read_text().splitlines() if line.split()[0] == 'stack']
    if not stack_ids:
        raise RuntimeError('no owned stack process recorded')
    root = stack_ids[-1]
    table = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True)
    rows = {}
    for line in table.splitlines():
        parts = line.strip().split(None, 2)
        if len(parts) == 3 and parts[0].isdigit() and parts[1].isdigit():
            rows[int(parts[0])] = (int(parts[1]), parts[2])
    def descendant(pid):
        seen = set()
        while pid in rows and pid not in seen:
            if pid == root: return True
            seen.add(pid); pid = rows[pid][0]
        return False
    targets = [pid for pid, (_, args) in rows.items()
               if descendant(pid) and re.search(r'nav2_bt_navigator/bt_navigator', args)]
    if len(targets) != 1:
        raise RuntimeError('expected exactly one owned bt_navigator, found ' + str(len(targets)))
    target = targets[0]
    os.kill(target, 19)
    try:
        t0 = time.monotonic(); request_mode('MANUAL')
        stopped = still_after(t0, .5, 1.0)
        rec, hist, secs = wait_mode('MANUAL', 45)
        status = mission_status(mid)
    finally:
        try: os.kill(target, 18)
        except ProcessLookupError: pass
"""
assert source.count(old)==1, 'upstream harness changed; review before running'
source=source.replace(old,new)
exec(compile(source,str(source_path),'exec'),{'__name__':'__main__','__file__':str(source_path)})
