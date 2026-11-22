"""R3 offline deployment tool tests (2026-11-23).

Pure offline laptop tests in temporary directories: NO SSH, NO robot access, NO repo mutation, NO ROS nodes.
Covers:
  - artifact generation (candidate vs approved, manifest, SHA-256, secret rejection, missing required files, dry-run)
  - single-file safe replace / verify / rollback (using temp copies of start_robot_agent_field.sh)
  - all 10 required fault-injection & rollback scenarios
  - backward compatibility of existing package preflight/backup/stage/replace/rollback actions + dry-run
"""
import hashlib
import json
import os
import shutil
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEPLOY_SCRIPT = ROOT / 'scripts' / 'limo_package_deploy_20261122.sh'
START_SCRIPT = ROOT / 'scripts' / 'start_robot_agent_field.sh'
AGENT_SRC = ROOT / 'agent' / 'limo_robot_agent'


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_deploy(*args, env=None):
    return subprocess.run(['bash', str(DEPLOY_SCRIPT), *map(str, args)], capture_output=True, text=True, timeout=60, env=env)


def tree_snapshot(root: Path) -> dict[str, tuple[str, int, str]]:
    """Capture complete directory tree state: relative path -> (kind, mode, sha256_or_target)."""
    snap = {}
    if not root.exists():
        return snap
    for p in sorted(root.rglob('*')):
        rel = str(p.relative_to(root))
        st = p.lstat()
        mode = stat.S_IMODE(st.st_mode)
        if p.is_symlink():
            snap[rel] = ('symlink', mode, os.readlink(p))
        elif p.is_file():
            snap[rel] = ('file', mode, sha256_file(p))
        elif p.is_dir():
            snap[rel] = ('dir', mode, '')
    return snap


class TestR3DeployTool(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix='r3_deploy_test_'))

    def tearDown(self):
        # Ensure any chmod-restricted test dirs are writable before cleanup
        for p in self.tmp.rglob('*'):
            try:
                if p.is_dir() and not p.is_symlink():
                    p.chmod(0o755)
            except OSError:
                pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _make_dummy_pkg(self, root: Path, pkg='dummy_pkg', content='v1'):
        pdir = root / pkg
        (pdir / pkg).mkdir(parents=True)
        (pdir / 'package.xml').write_text(f'<package><name>{pkg}</name></package>\n')
        (pdir / 'setup.py').write_text(f'from setuptools import setup\nsetup(name="{pkg}")\n')
        (pdir / pkg / '__init__.py').write_text(f'VERSION = "{content}"\n')
        return pdir

    def test_01_bash_syntax_and_start_script_compat(self):
        self.assertEqual(subprocess.run(['bash', '-n', str(DEPLOY_SCRIPT)]).returncode, 0)
        self.assertEqual(subprocess.run(['bash', '-n', str(START_SCRIPT)]).returncode, 0)
        # Verify start_robot_agent_field.sh argument guards (exits 2 before any ROS command)
        env = {k: v for k, v in os.environ.items() if k != 'STATE_DIR'}
        self.assertEqual(subprocess.run(['bash', str(START_SCRIPT)], env=env, capture_output=True).returncode, 2)
        self.assertEqual(subprocess.run(['bash', str(START_SCRIPT), 'ftp://bad', '/tmp/tok'], env=env, capture_output=True).returncode, 2)
        self.assertEqual(subprocess.run(['bash', str(START_SCRIPT), 'http://127.0.0.1:18290', '/tmp/tok'], env=env, capture_output=True).returncode, 2)
        env['STATE_DIR'] = 'relative/path'
        self.assertEqual(subprocess.run(['bash', str(START_SCRIPT), 'http://127.0.0.1:18290', '/tmp/tok'], env=env, capture_output=True).returncode, 2)

    def test_02_artifact_generation_dry_run_and_candidate_manifest(self):
        out_dry = self.tmp / 'art_dry'
        r = run_deploy('artifact', '--src-dir', AGENT_SRC, '--pkg', 'limo_robot_agent', '--out-dir', out_dry, '--stamp', 'r3_test', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('DRY-RUN artifact OK', r.stdout)
        self.assertFalse(out_dry.exists())

        out = self.tmp / 'art_real'
        r = run_deploy('artifact', '--src-dir', AGENT_SRC, '--pkg', 'limo_robot_agent', '--out-dir', out, '--stamp', 'r3_test')
        self.assertEqual(r.returncode, 0, r.stderr)
        tar = out / 'limo_robot_agent_candidate_r3_test.tar.gz'
        self.assertTrue(tar.is_file())
        self.assertTrue((out / 'FILES.sha256').is_file())
        self.assertTrue((out / 'TAR.sha256').is_file())
        manifest = json.loads((out / 'ARTIFACT.manifest.json').read_text())
        self.assertEqual(manifest['status'], 'candidate')
        self.assertIs(manifest['deployment_approved'], False)
        self.assertEqual(manifest['pkg'], 'limo_robot_agent')
        self.assertEqual(manifest['tar_sha256'], sha256_file(tar))
        self.assertGreaterEqual(manifest['file_count'], 25)
        self.assertIn('reproduce_cmd', manifest)
        # Duplicate artifact run with same stamp must abort (never overwrite)
        r_dup = run_deploy('artifact', '--src-dir', AGENT_SRC, '--pkg', 'limo_robot_agent', '--out-dir', out, '--stamp', 'r3_test')
        self.assertEqual(r_dup.returncode, 3)

    def test_03_artifact_rejects_secrets_and_missing_required_files(self):
        src = self._make_dummy_pkg(self.tmp / 'src_secret', 'dummy_pkg')
        (src / 'agent-pcu.token').write_text('secret-value')
        r = run_deploy('artifact', '--src-dir', src, '--pkg', 'dummy_pkg', '--out-dir', self.tmp / 'out_sec', '--stamp', 's1')
        self.assertEqual(r.returncode, 3)
        self.assertIn('forbidden secret', r.stderr)

        src_missing = self._make_dummy_pkg(self.tmp / 'src_missing', 'dummy_pkg')
        (src_missing / 'package.xml').unlink()
        r2 = run_deploy('artifact', '--src-dir', src_missing, '--pkg', 'dummy_pkg', '--out-dir', self.tmp / 'out_miss', '--stamp', 's1')
        self.assertEqual(r2.returncode, 3)
        self.assertIn('package.xml', r2.stderr)

    def test_04_single_file_replace_verify_reapply_and_rollback(self):
        target_dir = self.tmp / 'robot_tools'
        target_dir.mkdir()
        target = target_dir / 'start_robot_agent_field.sh'
        shutil.copy2(START_SCRIPT, target)
        target.chmod(0o700)
        orig_sha = sha256_file(target)

        new_src = self.tmp / 'new_start.sh'
        new_src.write_text(START_SCRIPT.read_text() + '\n# R3 test marker\n')
        new_src.chmod(0o700)
        new_sha_file = self.tmp / 'new_start.sh.sha256'
        new_sha_file.write_text(f'{sha256_file(new_src)}  start_robot_agent_field.sh\n')
        bdir = self.tmp / 'backups'

        # Dry-run leaves target and backup untouched
        r_dry = run_deploy('file-replace', '--target', target, '--src', new_src, '--src-sha', new_sha_file, '--backup-dir', bdir, '--stamp', 'st1', '--require-exec', '--dry-run')
        self.assertEqual(r_dry.returncode, 0, r_dry.stderr)
        self.assertEqual(sha256_file(target), orig_sha)
        self.assertFalse(bdir.exists())

        # Actual replace
        r = run_deploy('file-replace', '--target', target, '--src', new_src, '--src-sha', new_sha_file, '--backup-dir', bdir, '--stamp', 'st1', '--require-exec')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(sha256_file(target), sha256_file(new_src))
        self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o700)

        # Verify
        rv = run_deploy('file-verify', '--target', target, '--src-sha', new_sha_file, '--require-exec')
        self.assertEqual(rv.returncode, 0, rv.stderr)

        # Fault #7: duplicate run with same stamp refused
        r_dup = run_deploy('file-replace', '--target', target, '--src', new_src, '--src-sha', new_sha_file, '--backup-dir', bdir, '--stamp', 'st1', '--require-exec')
        self.assertEqual(r_dup.returncode, 3)

        # Fault #8: re-applying identical installed version with a new stamp succeeds with clear note
        r_re = run_deploy('file-replace', '--target', target, '--src', new_src, '--src-sha', new_sha_file, '--backup-dir', bdir, '--stamp', 'st2', '--require-exec')
        self.assertEqual(r_re.returncode, 0, r_re.stderr)
        self.assertIn('already identical to target', r_re.stdout)

        # Rollback st1 restores exact original checksum and mode
        r_rb = run_deploy('file-rollback', '--target', target, '--backup-dir', bdir, '--stamp', 'st1')
        self.assertEqual(r_rb.returncode, 0, r_rb.stderr)
        self.assertEqual(sha256_file(target), orig_sha)
        self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o700)

    def test_05_fault_injection_matrix_all_10_modes(self):
        """Case K: All 10 existing fault-injection modes remain green."""
        ws = self.tmp / 'ws'
        self._make_dummy_pkg(ws / 'src', 'dummy_pkg', 'v1')
        v2_src = self._make_dummy_pkg(self.tmp / 'v2_src', 'dummy_pkg', 'v2')
        art = self.tmp / 'art'
        bdir = self.tmp / 'bk'
        self.assertEqual(run_deploy('artifact', '--src-dir', v2_src, '--pkg', 'dummy_pkg', '--out-dir', art, '--stamp', 'v2').returncode, 0)
        tar = art / 'dummy_pkg_candidate_v2.tar.gz'
        tar_sha = art / 'TAR.sha256'
        files_sha = art / 'FILES.sha256'

        # 1. Missing artifact
        r1 = run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'f1',
                        '--tar', art / 'nope.tar.gz', '--tar-sha', tar_sha, '--files-sha', files_sha)
        self.assertEqual(r1.returncode, 3)

        # 2. Checksum mismatch (tampered TAR.sha256)
        bad_sha = self.tmp / 'BAD.sha256'
        bad_sha.write_text('0' * 64 + f'  {tar.name}\n')
        r2 = run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'f2',
                        '--tar', tar, '--tar-sha', bad_sha, '--files-sha', files_sha)
        self.assertEqual(r2.returncode, 3)
        self.assertIn('hash mismatch', r2.stderr)

        # 3. File permission error (non-executable replacement with --require-exec)
        tdir = self.tmp / 'perm_dir'; tdir.mkdir()
        tfile = tdir / 'tool.sh'; tfile.write_text('#!/bin/bash\necho v1\n'); tfile.chmod(0o700)
        nfile = self.tmp / 'noexec.sh'; nfile.write_text('#!/bin/bash\necho v2\n'); nfile.chmod(0o644)
        nsha = self.tmp / 'noexec.sha256'; nsha.write_text(f'{sha256_file(nfile)}  tool.sh\n')
        r3 = run_deploy('file-replace', '--target', tfile, '--src', nfile, '--src-sha', nsha, '--backup-dir', bdir, '--stamp', 'f3', '--require-exec')
        self.assertEqual(r3.returncode, 3)
        self.assertIn('execute permission', r3.stderr)
        self.assertIn('echo v1', tfile.read_text())

        # 4. Corrupt archive (forged TAR.sha256 matching corrupt bytes)
        corrupt_dir = self.tmp / 'corrupt'; corrupt_dir.mkdir()
        ctar = corrupt_dir / 'dummy_pkg_candidate_c.tar.gz'
        ctar.write_bytes(b'NOT A VALID GZIP ARCHIVE')
        csha = corrupt_dir / 'TAR.sha256'
        csha.write_text(f'{sha256_file(ctar)}  {ctar.name}\n')
        r4 = run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'f4',
                        '--tar', ctar, '--tar-sha', csha, '--files-sha', files_sha)
        self.assertEqual(r4.returncode, 3)
        self.assertIn('corrupt archive', r4.stderr)

        # 5. Missing target file
        nfile.chmod(0o700)
        r5 = run_deploy('file-replace', '--target', tdir / 'missing.sh', '--src', nfile, '--src-sha', nsha, '--backup-dir', bdir, '--stamp', 'f5', '--require-exec')
        self.assertEqual(r5.returncode, 3)
        self.assertIn('target file missing', r5.stderr)

        # 6. Mid-replace failure (unwritable target directory)
        tdir.chmod(0o555)
        r6 = run_deploy('file-replace', '--target', tfile, '--src', nfile, '--src-sha', nsha, '--backup-dir', bdir, '--stamp', 'f6', '--require-exec')
        self.assertEqual(r6.returncode, 3)
        self.assertIn('not writable', r6.stderr)
        tdir.chmod(0o755)
        self.assertIn('echo v1', tfile.read_text())

        # 9. Corrupt backup file refused during rollback (target stays untouched!)
        self.assertEqual(run_deploy('file-replace', '--target', tfile, '--src', nfile, '--src-sha', nsha, '--backup-dir', bdir, '--stamp', 'f9', '--require-exec').returncode, 0)
        bk_file = bdir / 'tool.sh.pre_f9'
        bk_file.write_text('CORRUPTED BACKUP BYTES')
        r9 = run_deploy('file-rollback', '--target', tfile, '--backup-dir', bdir, '--stamp', 'f9')
        self.assertEqual(r9.returncode, 3)
        self.assertIn('backup file corrupt', r9.stderr)
        self.assertIn('echo v2', tfile.read_text())

        # 10. Rollback failure (unwritable target directory)
        tfile2 = tdir / 'tool2.sh'; tfile2.write_text('#!/bin/bash\necho a\n'); tfile2.chmod(0o700)
        self.assertEqual(run_deploy('file-replace', '--target', tfile2, '--src', nfile, '--src-sha', nsha, '--backup-dir', bdir, '--stamp', 'f10', '--require-exec').returncode, 0)
        tdir.chmod(0o555)
        r10 = run_deploy('file-rollback', '--target', tfile2, '--backup-dir', bdir, '--stamp', 'f10')
        self.assertEqual(r10.returncode, 3)
        self.assertNotIn('file-rollback OK', r10.stdout)
        tdir.chmod(0o755)

    def test_06_package_backup_stage_replace_and_corrupt_backup_rollback_guard(self):
        ws = self.tmp / 'ws_pkg'
        orig_pkg = self._make_dummy_pkg(ws / 'src', 'dummy_pkg', 'v1')
        v1_init_sha = sha256_file(orig_pkg / 'dummy_pkg' / '__init__.py')
        v2_src = self._make_dummy_pkg(self.tmp / 'v2_pkg', 'dummy_pkg', 'v2')
        v2_init_sha = sha256_file(v2_src / 'dummy_pkg' / '__init__.py')
        art = self.tmp / 'art_pkg'
        bdir = self.tmp / 'bk_pkg'
        self.assertEqual(run_deploy('artifact', '--src-dir', v2_src, '--pkg', 'dummy_pkg', '--out-dir', art, '--stamp', 'v2').returncode, 0)
        tar, tar_sha, files_sha = art / 'dummy_pkg_candidate_v2.tar.gz', art / 'TAR.sha256', art / 'FILES.sha256'

        # preflight + backup + stage + replace
        self.assertEqual(run_deploy('preflight', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1').returncode, 0)
        self.assertEqual(run_deploy('backup', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1').returncode, 0)
        self.assertEqual(run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1', '--tar', tar, '--tar-sha', tar_sha, '--files-sha', files_sha).returncode, 0)
        self.assertEqual(run_deploy('replace', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1').returncode, 0)
        self.assertEqual(sha256_file(ws / 'src' / 'dummy_pkg' / 'dummy_pkg' / '__init__.py'), v2_init_sha)

        # Duplicate backup / stage / replace with same stamp refused
        self.assertEqual(run_deploy('backup', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1').returncode, 3)
        self.assertEqual(run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1', '--tar', tar, '--tar-sha', tar_sha, '--files-sha', files_sha).returncode, 3)
        self.assertEqual(run_deploy('replace', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1').returncode, 3)

        # Corrupt package backup tar -> rollback must abort and NOT report OK
        (bdir / 'dummy_pkg_src_pre_p1.tar.gz').write_bytes(b'corrupted tar')
        r_bad_rb = run_deploy('rollback', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 'p1')
        self.assertEqual(r_bad_rb.returncode, 3)
        self.assertIn('hash mismatch', r_bad_rb.stderr)
        self.assertNotIn('rollback OK', r_bad_rb.stdout)
        # Current v2 target in ws/src/dummy_pkg is still present and replaced v1 copy is preserved in bdir
        self.assertEqual(sha256_file(ws / 'src' / 'dummy_pkg' / 'dummy_pkg' / '__init__.py'), v2_init_sha)
        self.assertEqual(sha256_file(bdir / 'dummy_pkg_src_dir_replaced_p1' / 'dummy_pkg' / '__init__.py'), v1_init_sha)

    def test_07_rollback_atomicity_copy_failure_retry_checksum_and_interrupted(self):
        """Cases A, B, C, D: Rollback failure preserves existing target and allows retry with same stamp."""
        live = self.tmp / 'live_rb'; live.mkdir()
        bdir = self.tmp / 'bk_rb'; bdir.mkdir()
        target = live / 'start_robot_agent_field.sh'
        target.write_text('#!/usr/bin/env bash\necho old_version\n')
        target.chmod(0o750)
        old_sha = sha256_file(target)

        cand = self.tmp / 'cand.sh'
        cand.write_text('#!/usr/bin/env bash\necho candidate_version\n')
        cand.chmod(0o750)
        cand_sha_val = sha256_file(cand)
        cand_sha = self.tmp / 'cand.sha256'
        cand_sha.write_text(f'{cand_sha_val}  start_robot_agent_field.sh\n')

        r_rep = run_deploy('file-replace', '--target', target, '--src', cand, '--src-sha', cand_sha, '--backup-dir', bdir, '--stamp', 'rb_atom')
        self.assertEqual(r_rep.returncode, 0, r_rep.stderr)
        self.assertEqual(sha256_file(target), cand_sha_val)

        # Case A: Rollback copy failure (via injected failing cp in PATH, works even as root) -> existing target preserved
        fakebin_fail = self.tmp / 'fakebin_fail'; fakebin_fail.mkdir()
        fakecp_fail = fakebin_fail / 'cp'
        fakecp_fail.write_text('#!/usr/bin/env bash\necho simulated cp failure >&2\nexit 1\n')
        fakecp_fail.chmod(0o755)
        env_fail = dict(os.environ, PATH=f"{fakebin_fail}:{os.environ['PATH']}")

        r_fail = run_deploy('file-rollback', '--target', target, '--backup-dir', bdir, '--stamp', 'rb_atom', env=env_fail)
        self.assertEqual(r_fail.returncode, 3)
        self.assertTrue(target.is_file(), 'Target was lost after rollback copy failure!')
        self.assertEqual(sha256_file(target), cand_sha_val)
        self.assertFalse((bdir / 'start_robot_agent_field.sh.failed_rb_atom').exists())

        # Case C: Rollback staged temp file checksum verification failure (cp writes corrupted bytes to staging file) -> swap refused, target preserved
        fakebin_corrupt = self.tmp / 'fakebin_corrupt'; fakebin_corrupt.mkdir()
        fakecp_corrupt = fakebin_corrupt / 'cp'
        fakecp_corrupt.write_text('#!/usr/bin/env bash\nshift $(( $# - 1 ))\necho "CORRUPTED_STAGE" > "$1"\nexit 0\n')
        fakecp_corrupt.chmod(0o755)
        env_corrupt = dict(os.environ, PATH=f"{fakebin_corrupt}:{os.environ['PATH']}")

        r_corrupt = run_deploy('file-rollback', '--target', target, '--backup-dir', bdir, '--stamp', 'rb_atom', env=env_corrupt)
        self.assertEqual(r_corrupt.returncode, 3)
        self.assertIn('checksum verification failed', r_corrupt.stderr)
        self.assertTrue(target.is_file(), 'Target was lost after staged checksum mismatch!')
        self.assertEqual(sha256_file(target), cand_sha_val)

        # Case D: Interrupted/aborted swap (mv fails after staging and archiving) -> existing target remains valid, FAILED archive cleaned up
        fakebin_mv = self.tmp / 'fakebin_mv'; fakebin_mv.mkdir()
        fakemv = fakebin_mv / 'mv'
        fakemv.write_text('#!/usr/bin/env bash\necho simulated mv failure >&2\nexit 1\n')
        fakemv.chmod(0o755)
        env_mv = dict(os.environ, PATH=f"{fakebin_mv}:{os.environ['PATH']}")

        r_mv = run_deploy('file-rollback', '--target', target, '--backup-dir', bdir, '--stamp', 'rb_atom', env=env_mv)
        self.assertEqual(r_mv.returncode, 3)
        self.assertTrue(target.is_file())
        self.assertEqual(sha256_file(target), cand_sha_val)
        self.assertFalse((bdir / 'start_robot_agent_field.sh.failed_rb_atom').exists())

        # Case B: Retry file-rollback with the SAME stamp after previous failures succeeds cleanly
        r_retry = run_deploy('file-rollback', '--target', target, '--backup-dir', bdir, '--stamp', 'rb_atom')
        self.assertEqual(r_retry.returncode, 0, r_retry.stderr)
        self.assertTrue(target.is_file())
        self.assertEqual(sha256_file(target), old_sha)
        self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o750)

    def test_08_dry_run_zero_mutation_across_all_actions(self):
        """Cases E & F: verify --dry-run and all other --dry-run actions cause 0 filesystem changes."""
        ws = self.tmp / 'ws_dry'
        self._make_dummy_pkg(ws / 'src', 'dummy_pkg', 'v1')
        (ws / 'install').mkdir(parents=True)
        sentinel = ws / 'setup_sourced_during_dry_run'
        (ws / 'install' / 'setup.bash').write_text(f'touch {sentinel}\n')

        art = self.tmp / 'art_dry_src'
        self.assertEqual(run_deploy('artifact', '--src-dir', ws / 'src' / 'dummy_pkg', '--pkg', 'dummy_pkg', '--out-dir', art, '--stamp', 's0').returncode, 0)
        tar = art / 'dummy_pkg_candidate_s0.tar.gz'
        tar_sha = art / 'TAR.sha256'
        files_sha = art / 'FILES.sha256'

        # Pre-populate backup and stage fixtures so replace/rollback/file-rollback dry-runs reach their full dry-run path
        bdir = self.tmp / 'bk_dry'
        self.assertEqual(run_deploy('backup', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep').returncode, 0)
        self.assertEqual(run_deploy('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep',
                                    '--tar', tar, '--tar-sha', tar_sha, '--files-sha', files_sha).returncode, 0)

        tdir = self.tmp / 'f_dry'; tdir.mkdir()
        tfile = tdir / 'tool.sh'; tfile.write_text('#!/bin/bash\necho v1\n'); tfile.chmod(0o700)
        tsha = self.tmp / 'tool.sha256'; tsha.write_text(f'{sha256_file(tfile)}  tool.sh\n')
        (bdir / 'tool.sh.pre_s_prep').write_text('#!/bin/bash\necho v0\n')
        (bdir / 'tool.sh.pre_s_prep.sha256').write_text(f"{sha256_file(bdir / 'tool.sh.pre_s_prep')}  tool.sh\n")
        (bdir / 'tool.sh.pre_s_prep.mode').write_text('700\n')

        before = tree_snapshot(self.tmp)

        # Case E: verify --dry-run must NOT source install/setup.bash or create sentinel
        r_ver = run_deploy('verify', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep', '--files-sha', files_sha, '--dry-run')
        self.assertEqual(r_ver.returncode, 0, r_ver.stderr)
        self.assertFalse(sentinel.exists(), 'verify --dry-run sourced install/setup.bash and mutated filesystem!')

        # Case F: Every action in --dry-run mode
        dry_cmds = [
            ('artifact', '--src-dir', ws / 'src' / 'dummy_pkg', '--pkg', 'dummy_pkg', '--out-dir', self.tmp / 'never_created', '--stamp', 's_new', '--dry-run'),
            ('preflight', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_new', '--dry-run'),
            ('backup', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_new', '--dry-run'),
            ('stage', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_new', '--tar', tar, '--tar-sha', tar_sha, '--files-sha', files_sha, '--dry-run'),
            ('replace', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep', '--dry-run'),
            ('build', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep', '--dry-run'),
            ('verify', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep', '--files-sha', files_sha, '--dry-run'),
            ('rollback', '--ws', ws, '--pkg', 'dummy_pkg', '--backup-dir', bdir, '--stamp', 's_prep', '--dry-run'),
            ('file-replace', '--target', tfile, '--src', tfile, '--src-sha', tsha, '--backup-dir', bdir, '--stamp', 's_new', '--require-exec', '--dry-run'),
            ('file-verify', '--target', tfile, '--src-sha', tsha, '--require-exec', '--dry-run'),
            ('file-rollback', '--target', tfile, '--backup-dir', bdir, '--stamp', 's_prep', '--dry-run'),
        ]
        for cmd in dry_cmds:
            res = run_deploy(*cmd)
            self.assertEqual(res.returncode, 0, f'Dry-run failed for {cmd[0]}: {res.stderr}')

        after = tree_snapshot(self.tmp)
        self.assertEqual(before, after, 'Filesystem tree changed during --dry-run execution!')

    def test_09_symlink_target_and_parent_symlink_traversal_rejected(self):
        """Cases G & H: Symbolic link target and symbolic link ancestor directory are rejected fail-closed."""
        live = self.tmp / 'live_sym'; live.mkdir()
        outside = self.tmp / 'outside_sym'; outside.mkdir()
        bdir = self.tmp / 'bk_sym'; bdir.mkdir()

        real_file = outside / 'original.sh'
        real_file.write_text('#!/usr/bin/env bash\necho original_outside\n')
        real_file.chmod(0o750)
        orig_sha = sha256_file(real_file)

        cand = self.tmp / 'cand_sym.sh'
        cand.write_text('#!/usr/bin/env bash\necho candidate_payload\n')
        cand.chmod(0o750)
        cand_sha = self.tmp / 'cand_sym.sha256'
        cand_sha.write_text(f'{sha256_file(cand)}  original.sh\n')

        # Case G: Target itself is a symbolic link
        sym_target = live / 'tool_link.sh'
        sym_target.symlink_to(real_file)
        r_g = run_deploy('file-replace', '--target', sym_target, '--src', cand, '--src-sha', cand_sha, '--backup-dir', bdir, '--stamp', 'sym_g')
        self.assertEqual(r_g.returncode, 3)
        self.assertIn('symbolic link', r_g.stderr)
        self.assertTrue(sym_target.is_symlink(), 'Symlink target was overwritten with a regular file!')
        self.assertEqual(sha256_file(real_file), orig_sha)
        self.assertEqual(stat.S_IMODE(real_file.stat().st_mode), 0o750)

        # file-verify and file-rollback on a symlink target must also reject fail-closed
        self.assertEqual(run_deploy('file-verify', '--target', sym_target, '--src-sha', cand_sha).returncode, 3)
        self.assertEqual(run_deploy('file-rollback', '--target', sym_target, '--backup-dir', bdir, '--stamp', 'sym_g').returncode, 3)

        # Case H: Parent directory is a symbolic link pointing outside live/
        parent_link = live / 'alias_dir'
        parent_link.symlink_to(outside)
        traversal_target = parent_link / 'original.sh'
        r_h = run_deploy('file-replace', '--target', traversal_target, '--src', cand, '--src-sha', cand_sha, '--backup-dir', bdir, '--stamp', 'sym_h')
        self.assertEqual(r_h.returncode, 3)
        self.assertIn('symbolic link', r_h.stderr)
        self.assertEqual(sha256_file(real_file), orig_sha, 'Outside referent file was modified through parent symlink!')

        # Package source with a symlink inside must also be rejected by artifact
        pkg_sym = self._make_dummy_pkg(self.tmp / 'pkg_with_sym', 'dummy_pkg')
        (pkg_sym / 'leak.py').symlink_to(real_file)
        r_pkg_sym = run_deploy('artifact', '--src-dir', pkg_sym, '--pkg', 'dummy_pkg', '--out-dir', self.tmp / 'out_pkg_sym', '--stamp', 'sym_p')
        self.assertEqual(r_pkg_sym.returncode, 3)
        self.assertIn('symbolic link', r_pkg_sym.stderr)

    def test_10_artifact_determinism_and_allowlist_blocklist_policy(self):
        """Cases I & J: Bit-for-bit deterministic tar.gz across time/paths, strict file policy, and manifest deployment_approved=false."""
        import time
        src_a = self.tmp / 'path_a' / 'limo_robot_agent'
        src_b = self.tmp / 'path_b' / 'limo_robot_agent'
        shutil.copytree(AGENT_SRC, src_a)
        shutil.copytree(AGENT_SRC, src_b)

        out1 = self.tmp / 'det_out1'
        out2 = self.tmp / 'det_out2'
        r1 = run_deploy('artifact', '--src-dir', src_a, '--pkg', 'limo_robot_agent', '--out-dir', out1, '--stamp', 'det')
        self.assertEqual(r1.returncode, 0, r1.stderr)
        time.sleep(1.1)
        r2 = run_deploy('artifact', '--src-dir', src_b, '--pkg', 'limo_robot_agent', '--out-dir', out2, '--stamp', 'det')
        self.assertEqual(r2.returncode, 0, r2.stderr)

        # Case I: Identical FILES.sha256 and bit-for-bit identical .tar.gz SHA-256 across different source directories and timestamps
        self.assertEqual(sha256_file(out1 / 'FILES.sha256'), sha256_file(out2 / 'FILES.sha256'))
        self.assertEqual(
            sha256_file(out1 / 'limo_robot_agent_candidate_det.tar.gz'),
            sha256_file(out2 / 'limo_robot_agent_candidate_det.tar.gz'),
        )

        # Case J: kit.env, .env.local, *.token, and non-allowlisted binary/unknown files are rejected
        for forbidden_name in ('kit.env', '.env.local', 'robot.token', 'unexpected.bin'):
            bad_src = self._make_dummy_pkg(self.tmp / f'bad_{forbidden_name}', 'dummy_pkg')
            (bad_src / forbidden_name).write_text('forbidden content\n')
            r_bad = run_deploy('artifact', '--src-dir', bad_src, '--pkg', 'dummy_pkg', '--out-dir', self.tmp / f'out_{forbidden_name}', '--stamp', 's1')
            self.assertEqual(r_bad.returncode, 3, f'Failed to reject {forbidden_name}')

        # Even if --status approved is passed on CLI, manifest records deployment_approved: false
        out_app = self.tmp / 'out_approved_label'
        r_app = run_deploy('artifact', '--src-dir', src_a, '--pkg', 'limo_robot_agent', '--out-dir', out_app, '--stamp', 'app1', '--status', 'approved')
        self.assertEqual(r_app.returncode, 0, r_app.stderr)
        manifest = json.loads((out_app / 'ARTIFACT.manifest.json').read_text())
        self.assertEqual(manifest['status'], 'approved')
        self.assertIs(manifest['deployment_approved'], False)

    def test_11_package_rollback_normal_compensation_and_double_failure_recovery(self):
        """R3 Cases A, B, C, D, E: Normal package rollback, swap failure + verified compensation,
        double failure (swap + compensation fail) with exact ABSENT/FAILED diagnostics, and recovery from FAILED."""
        # Case A: Normal end-to-end package rollback with real limo_robot_agent package
        ws_a = self.tmp / 'ws_pkg_rb_ok'
        pkg_a = ws_a / 'src' / 'limo_robot_agent'
        shutil.copytree(AGENT_SRC, pkg_a)
        bk_a = self.tmp / 'bk_pkg_rb_ok'
        init_a = pkg_a / 'limo_robot_agent' / '__init__.py'
        orig_init_sha = sha256_file(init_a)
        r_bk_a = run_deploy('backup', '--ws', ws_a, '--pkg', 'limo_robot_agent', '--backup-dir', bk_a, '--stamp', 'ok1')
        self.assertEqual(r_bk_a.returncode, 0, r_bk_a.stderr)
        # Mutate live package as a simulated bad deploy
        init_a.write_text('# bad deployed version\n')
        bad_init_sha = sha256_file(init_a)
        self.assertNotEqual(orig_init_sha, bad_init_sha)
        r_rb_a = run_deploy('rollback', '--ws', ws_a, '--pkg', 'limo_robot_agent', '--backup-dir', bk_a, '--stamp', 'ok1')
        self.assertEqual(r_rb_a.returncode, 0, r_rb_a.stdout + '\n' + r_rb_a.stderr)
        self.assertIn('rollback OK: restored', r_rb_a.stdout)
        self.assertEqual(sha256_file(init_a), orig_init_sha)
        # Replaced bad version is preserved in FAILED archive
        failed_a = bk_a / 'limo_robot_agent_src_dir_failed_ok1' / 'limo_robot_agent' / '__init__.py'
        self.assertTrue(failed_a.is_file())
        self.assertEqual(sha256_file(failed_a), bad_init_sha)

        # Setup mv shim for Cases B, C, D, E (matching Codex r3-inject.py)
        shim = self.tmp / 'mv_shim'; shim.mkdir()
        mv = shim / 'mv'
        mv.write_text('''#!/bin/bash
printf '%s -> %s\\n' "$1" "$2" >> "$AUDIT_MV_LOG"
case "$1" in
  */.rb_stage_*/*) exit 72;;
  *_src_dir_failed_*) if [ "$AUDIT_DOUBLE" = yes ]; then exit 73; fi;;
esac
exec /usr/bin/mv "$@"
''')
        mv.chmod(0o700)
        shim_path = f"{shim}:{os.environ['PATH']}"

        # Case B: Staged -> SRC fails, compensation FAILED -> SRC succeeds and is verified
        d_b = self.tmp / 'pkg_comp_ok'
        ws_b = d_b / 'ws'
        pkg_b = self._make_dummy_pkg(ws_b / 'src', 'demo', 'backup_v1')
        bk_b = d_b / 'archives'
        cli_b = ('--ws', ws_b, '--pkg', 'demo', '--stamp', 'q', '--backup-dir', bk_b)
        self.assertEqual(run_deploy('backup', *cli_b).returncode, 0)
        (pkg_b / 'demo' / '__init__.py').write_text('VERSION = "current_v2"\n')
        curr_b_sha = sha256_file(pkg_b / 'demo' / '__init__.py')
        r_b = run_deploy(
            'rollback', *cli_b,
            env=dict(os.environ, PATH=shim_path, AUDIT_MV_LOG=str(d_b / 'mv.log'), AUDIT_DOUBLE='no'),
        )
        self.assertEqual(r_b.returncode, 3)
        self.assertTrue(pkg_b.is_dir())
        self.assertEqual(sha256_file(pkg_b / 'demo' / '__init__.py'), curr_b_sha)
        self.assertFalse((bk_b / 'demo_src_dir_failed_q').exists())
        self.assertIn('compensation succeeded and verified', r_b.stderr)
        self.assertNotIn('rollback OK', r_b.stdout)

        # Cases C & D: Staged -> SRC fails AND compensation FAILED -> SRC also fails
        d_c = self.tmp / 'pkg_comp_fail'
        ws_c = d_c / 'ws'
        pkg_c = self._make_dummy_pkg(ws_c / 'src', 'demo', 'backup_v1')
        bk_c = d_c / 'archives'
        cli_c = ('--ws', ws_c, '--pkg', 'demo', '--stamp', 'q', '--backup-dir', bk_c)
        self.assertEqual(run_deploy('backup', *cli_c).returncode, 0)
        (pkg_c / 'demo' / '__init__.py').write_text('VERSION = "current_v2"\n')
        curr_c_sha = sha256_file(pkg_c / 'demo' / '__init__.py')
        r_c = run_deploy(
            'rollback', *cli_c,
            env=dict(os.environ, PATH=shim_path, AUDIT_MV_LOG=str(d_c / 'mv.log'), AUDIT_DOUBLE='yes'),
        )
        failed_c = bk_c / 'demo_src_dir_failed_q'
        self.assertEqual(r_c.returncode, 3)
        self.assertFalse(pkg_c.exists(), 'SRC should be absent in double-failure scenario')
        self.assertTrue(failed_c.is_dir(), 'FAILED archive must be preserved and never deleted')
        self.assertEqual(sha256_file(failed_c / 'demo' / '__init__.py'), curr_c_sha)
        # Must NEVER falsely claim "restored previous" or "rollback OK"
        combined_c = r_c.stdout + '\n' + r_c.stderr
        self.assertNotIn('restored previous', combined_c)
        self.assertNotIn('rollback OK', combined_c)
        self.assertIn('SRC is ABSENT', combined_c)
        self.assertIn(str(failed_c), combined_c)
        self.assertIn('RECOVERY:', combined_c)

        # Case E: Independent recovery from preserved FAILED archive after double failure
        shutil.move(str(failed_c), str(pkg_c))
        self.assertTrue(pkg_c.is_dir())
        self.assertEqual(sha256_file(pkg_c / 'demo' / '__init__.py'), curr_c_sha)
        self.assertFalse(failed_c.exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
