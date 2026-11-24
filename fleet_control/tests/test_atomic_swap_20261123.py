"""R3 follow-up (2026-11-23): atomic package swap - SRC is never absent, every failure leaves SRC present and reports the state.

Real renameat2(RENAME_EXCHANGE) on a temp directory (tmpfs/ext4); failures are injected through the exchange and verify hooks.
"""
import hashlib
import os
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import atomic_swap_20261123 as sw


def make_tree(root, files):
    for rel, text in files.items():
        p = Path(root, rel); p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)


def write_sums(path, root, pkg):
    lines = [f'{hashlib.sha256(Path(root, r).read_bytes()).hexdigest()}  {pkg}/{r}' for r in sorted(sw.tree_hashes(root))]
    Path(path).write_text('\n'.join(lines) + '\n')


class AtomicSwap(unittest.TestCase):
    def setUp(self):
        self.t = tempfile.TemporaryDirectory(); self.addCleanup(self.t.cleanup)
        self.ws = Path(self.t.name, 'ws/src'); self.ws.mkdir(parents=True)
        self.src = self.ws / 'pkg'; self.stage = self.ws / '.stage_pkg'
        make_tree(self.src, {'a.py': 'old\n', 'setup.py': 'S\n'})
        make_tree(self.stage, {'a.py': 'new\n', 'setup.py': 'S\n', 'b.py': 'added\n'})
        self.sums = Path(self.t.name, 'FILES.sha256'); write_sums(self.sums, self.stage, 'pkg')
        self.old_hashes = sw.tree_hashes(self.src); self.new_hashes = sw.tree_hashes(self.stage)

    def run_swap(self, **kw):
        return sw.swap_in(str(self.src), str(self.stage), str(self.sums), force_running=True, **kw)

    def test_swap_installs_new_tree_and_preserves_old(self):
        state, _ = self.run_swap()
        self.assertEqual(state, 'SWAPPED')
        self.assertEqual(sw.tree_hashes(self.src), self.new_hashes)
        self.assertEqual(sw.tree_hashes(self.stage), self.old_hashes)            # nothing deleted

    def test_rollback_with_an_r3_backup_hash_list_uses_dot_slash_paths(self):
        """The R3 backup lists files as './relative/path' (found in the 2026-11-23 rehearsal: the rollback was refused)."""
        dot = Path(self.t.name, 'backup.files.sha256')
        dot.write_text('\n'.join(f'{h}  ./{r}' for r, h in sorted(self.old_hashes.items())) + '\n')
        backup_tree = self.ws / '.backup_pkg'
        make_tree(backup_tree, {'a.py': 'old\n', 'setup.py': 'S\n'})
        make_tree(self.src, {'pkg/inner.py': 'x\n'}); make_tree(backup_tree, {'pkg/inner.py': 'x\n'})     # a sub-directory named like the package (limo_robot_agent/limo_robot_agent)
        self.old_hashes = sw.tree_hashes(self.src)
        dot.write_text('\n'.join(f'{h}  ./{r}' for r, h in sorted(self.old_hashes.items())) + '\n')
        state, _ = sw.swap_in(str(self.src), str(backup_tree), str(dot), force_running=True)
        self.assertEqual(state, 'SWAPPED'); self.assertEqual(sw.tree_hashes(self.src), self.old_hashes)

    def test_src_is_never_absent_during_the_swap(self):
        seen_missing = []
        stop = threading.Event()
        def watcher():
            while not stop.is_set():
                if not os.path.isdir(self.src) or not (self.src / 'setup.py').exists():
                    seen_missing.append(True)
        th = threading.Thread(target=watcher); th.start()
        try:
            for _ in range(300):                                                   # swap back and forth many times
                sw.exchange(str(self.src), str(self.stage))
        finally:
            stop.set(); th.join()
        self.assertEqual(seen_missing, [])

    def test_unsupported_exchange_changes_nothing(self):
        def broken(a, b): raise OSError(22, 'not supported')
        state, msg = self.run_swap(exchange_fn=broken)
        self.assertEqual(state, 'UNCHANGED'); self.assertIn('nothing changed', msg)
        self.assertEqual(sw.tree_hashes(self.src), self.old_hashes)

    def test_corrupt_stage_is_refused_before_anything_moves(self):
        (self.stage / 'a.py').write_text('tampered\n')
        state, _ = self.run_swap()
        self.assertEqual(state, 'UNCHANGED'); self.assertEqual(sw.tree_hashes(self.src), self.old_hashes)

    def test_post_swap_verification_failure_swaps_back(self):
        calls = {'n': 0}
        def verify(root, files_sha, pkg):
            calls['n'] += 1
            return calls['n'] == 1                                                  # stage verifies, installed tree does not
        state, _ = self.run_swap(verify_fn=verify)
        self.assertEqual(state, 'ROLLED_BACK')
        self.assertEqual(sw.tree_hashes(self.src), self.old_hashes)

    def test_double_failure_still_leaves_src_present(self):
        real, n = sw.exchange, {'n': 0}
        def flaky(a, b):
            n['n'] += 1
            if n['n'] == 2: raise OSError(5, 'injected swap-back failure')
            return real(a, b)
        calls = {'n': 0}
        def verify(root, files_sha, pkg):
            calls['n'] += 1
            return calls['n'] == 1
        state, msg = self.run_swap(exchange_fn=flaky, verify_fn=verify)
        self.assertEqual(state, 'SWAPPED_UNVERIFIED')
        self.assertTrue(self.src.is_dir() and (self.src / 'setup.py').exists())    # the R3 failure mode: SRC absent. Not possible here.
        self.assertEqual(sw.tree_hashes(self.stage), self.old_hashes)               # previous tree preserved
        self.assertIn(str(self.stage), msg)

    def test_symlinked_src_or_stage_is_refused(self):
        link = self.ws / 'link'; link.symlink_to(self.stage)
        state, _ = sw.swap_in(str(self.src), str(link), str(self.sums), force_running=True)
        self.assertEqual(state, 'UNCHANGED')

    def test_different_filesystems_are_refused(self):
        real = os.stat
        def fake(p, *a, **k):
            r = real(p, *a, **k)
            return os.stat_result((r.st_mode, r.st_ino, r.st_dev + (1 if str(p) == str(self.stage) else 0)) + tuple(r)[3:])
        orig = sw.os.stat; sw.os.stat = fake
        try:
            state, _ = self.run_swap()
        finally:
            sw.os.stat = orig
        self.assertEqual(state, 'UNCHANGED')

    def test_running_robot_processes_block_the_swap(self):
        orig = sw.robot_processes; sw.robot_processes = lambda: ['robot_agent --ros-args']
        try:
            state, msg = sw.swap_in(str(self.src), str(self.stage), str(self.sums))
        finally:
            sw.robot_processes = orig
        self.assertEqual(state, 'UNCHANGED'); self.assertIn('stop them first', msg)

    def test_cli_replace_and_exit_codes(self):
        script = Path(sw.__file__)
        r = subprocess.run([sys.executable, str(script), 'replace', '--src', str(self.src), '--stage', str(self.stage),
                            '--files-sha', str(self.sums), '--force-running'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(sw.tree_hashes(self.src), self.new_hashes)
        (self.stage / 'a.py').write_text('x\n')
        r = subprocess.run([sys.executable, str(script), 'rollback', '--src', str(self.src), '--stage', str(self.stage),
                            '--files-sha', str(self.sums), '--force-running'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 3)                                          # stage no longer matches its list


if __name__ == '__main__':
    unittest.main()
