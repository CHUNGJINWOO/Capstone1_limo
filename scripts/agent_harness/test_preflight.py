#!/usr/bin/env python3
"""Unit tests for Agent Development Harness v0 (M0) read-only Preflight CLI."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import preflight  # noqa: E402


def _git(cwd: Path, *args: str) -> str:
    proc = subprocess.run(
        ["git", "-C", str(cwd), *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return proc.stdout.strip()


def _init_synthetic_repo(root: Path) -> str:
    """Create a minimal synthetic git repo with initial commit and return HEAD SHA."""
    root.mkdir(parents=True, exist_ok=True)
    _git(root, "init", "-b", "feat/agent-harness-m0-test")
    _git(root, "config", "user.name", "Test User")
    _git(root, "config", "user.email", "test@example.invalid")
    (root / "scripts" / "agent_harness").mkdir(parents=True, exist_ok=True)
    (root / "scripts" / "agent_harness" / "demo.py").write_text("print('v1')\n", encoding="utf-8")
    (root / "fleet_control" / "server" / "fleet_server" / "static").mkdir(parents=True, exist_ok=True)
    (root / "fleet_control" / "server" / "fleet_server" / "static" / "app.js").write_text("// v1\n", encoding="utf-8")
    (root / "fleet_control" / "agent" / "limo_robot_agent").mkdir(parents=True, exist_ok=True)
    (root / "fleet_control" / "agent" / "limo_robot_agent" / "core.py").write_text("# v1\n", encoding="utf-8")
    _git(root, "add", ".")
    _git(root, "commit", "-m", "initial fixture")
    return _git(root, "rev-parse", "HEAD")


def _clean_procs():
    return True, [], None


def _clean_ports():
    return True, [], None


class TestAgentHarnessPreflight(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory(prefix="agent_harness_test_")
        self.tmp_path = Path(self.tmp.name)
        self.repo = self.tmp_path / "repo"
        self.base_sha = _init_synthetic_repo(self.repo)
        self.clean_env = {"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "171", "ROS_LOCALHOST_ONLY": "1"}

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_01_clean_worktree(self) -> None:
        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-clean",
            scope="agent_harness",
            planned_files=["scripts/agent_harness/demo.py"],
            base_commit=self.base_sha,
            expected_branch="feat/agent-harness-m0-test",
            requested_domain=171,
            requested_ports=[29751],
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_PASS)
        self.assertEqual(report["checks"]["git"]["status"], preflight.STATUS_PASS)
        self.assertEqual(report["checks"]["scope"]["status"], preflight.STATUS_PASS)
        self.assertEqual(report["checks"]["ros_environment"]["status"], preflight.STATUS_PASS)
        self.assertEqual(report["checks"]["dangerous_operations"]["status"], preflight.STATUS_PASS)

    def test_02_dirty_worktree_does_not_fail_solely_for_dirty(self) -> None:
        (self.repo / "scripts" / "agent_harness" / "demo.py").write_text("print('v2')\n", encoding="utf-8")
        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-dirty",
            scope="agent_harness",
            planned_files=["scripts/agent_harness/new_file.py"],
            base_commit=self.base_sha,
            expected_branch="feat/agent-harness-m0-test",
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_WARN)
        self.assertEqual(report["checks"]["git"]["status"], preflight.STATUS_WARN)
        self.assertIn(
            "scripts/agent_harness/demo.py",
            report["checks"]["scope"]["pre_existing_dirty_outside_planned"],
        )

    def test_03_staged_and_unstaged_mixed(self) -> None:
        # File 1: staged only
        p1 = self.repo / "scripts" / "agent_harness" / "staged_only.py"
        p1.write_text("# staged\n", encoding="utf-8")
        _git(self.repo, "add", "scripts/agent_harness/staged_only.py")
        # File 2: both staged and unstaged
        p2 = self.repo / "scripts" / "agent_harness" / "demo.py"
        p2.write_text("print('staged')\n", encoding="utf-8")
        _git(self.repo, "add", "scripts/agent_harness/demo.py")
        p2.write_text("print('unstaged-after-stage')\n", encoding="utf-8")
        # File 3: unstaged only
        p3 = self.repo / "fleet_control" / "server" / "fleet_server" / "static" / "app.js"
        p3.write_text("// unstaged\n", encoding="utf-8")

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-mixed",
            scope="agent_harness",
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        cf = report["checks"]["git"]["changed_files"]
        self.assertEqual(cf["staged"], ["scripts/agent_harness/staged_only.py"])
        self.assertEqual(cf["staged_and_unstaged"], ["scripts/agent_harness/demo.py"])
        self.assertEqual(cf["unstaged"], ["fleet_control/server/fleet_server/static/app.js"])

    def test_04_untracked_files_and_unregistered_scope(self) -> None:
        (self.repo / "notes_unregistered.txt").write_text("hello\n", encoding="utf-8")
        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-untracked",
            scope="agent_harness",
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        cf = report["checks"]["git"]["changed_files"]
        self.assertEqual(cf["untracked"], ["notes_unregistered.txt"])
        cls = {c["path"]: c for c in report["checks"]["scope"]["file_classifications"]}
        self.assertEqual(cls["notes_unregistered.txt"]["kind"], "UNREGISTERED")
        self.assertIsNone(cls["notes_unregistered.txt"]["owner"])
        self.assertEqual(report["checks"]["scope"]["status"], preflight.STATUS_WARN)

    def test_05_other_agent_scope_conflict_blocked(self) -> None:
        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-conflict",
            scope="web_ui",
            planned_files=["fleet_control/agent/limo_robot_agent/core.py"],
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_BLOCKED)
        self.assertEqual(report["checks"]["scope"]["status"], preflight.STATUS_BLOCKED)
        codes = {f["code"] for f in report["checks"]["scope"]["findings"]}
        self.assertIn("OTHER_AGENT_SCOPE_CONFLICT", codes)

    def test_06_cross_worktree_uncommitted_collision(self) -> None:
        sib = self.tmp_path / "sibling_wt"
        _git(self.repo, "worktree", "add", "-b", "feat/sibling", str(sib), self.base_sha)
        (sib / "scripts" / "agent_harness" / "demo.py").write_text("print('sibling')\n", encoding="utf-8")

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-sib-collision",
            scope="agent_harness",
            planned_files=["scripts/agent_harness/demo.py"],
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_BLOCKED)
        scope_codes = {f["code"] for f in report["checks"]["scope"]["findings"]}
        self.assertIn("CROSS_WORKTREE_UNCOMMITTED_COLLISION", scope_codes)
        danger_actions = {f["action"] for f in report["checks"]["dangerous_operations"]["findings"]}
        self.assertIn("overwrite_other_agent_uncommitted", danger_actions)

    def test_07_branch_and_head_mismatch(self) -> None:
        # Advance HEAD by 1 commit so HEAD != base_sha
        (self.repo / "scripts" / "agent_harness" / "demo.py").write_text("print('v2')\n", encoding="utf-8")
        _git(self.repo, "add", "scripts/agent_harness/demo.py")
        _git(self.repo, "commit", "-m", "advance head")

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-mismatch",
            scope="agent_harness",
            base_commit=self.base_sha,
            expected_branch="feat/different-branch",
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        codes = {f["code"] for f in report["checks"]["git"]["findings"]}
        self.assertIn("BRANCH_MISMATCH", codes)
        self.assertIn("HEAD_BASE_MISMATCH", codes)
        self.assertEqual(report["checks"]["git"]["status"], preflight.STATUS_WARN)

    def test_08_ros_domain_and_protected_port_collision(self) -> None:
        def busy_procs():
            return (
                True,
                [{"pid": 99999, "domain_id": 195, "port": 29444, "cmd_preview": "ros2 --ros-domain-id 195"}],
                None,
            )

        def busy_ports():
            return True, [18190, 29444], None

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-ros-collision",
            scope="agent_harness",
            base_commit=self.base_sha,
            requested_domain=195,
            requested_ports=[18190, 29444],
            env=self.clean_env,
            process_inspector=busy_procs,
            port_inspector=busy_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_BLOCKED)
        codes = {f["code"] for f in report["checks"]["ros_environment"]["findings"]}
        self.assertIn("ROS_DOMAIN_COLLISION_ACTIVE", codes)
        self.assertIn("PROTECTED_PORT_REQUESTED", codes)
        self.assertIn("PORT_ALREADY_IN_USE", codes)

    def test_09_process_inspection_failure_returns_unknown(self) -> None:
        def failing_procs():
            return False, [], "simulated ps permission error"

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-ps-fail",
            scope="agent_harness",
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=failing_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["checks"]["ros_environment"]["status"], preflight.STATUS_UNKNOWN)
        self.assertEqual(report["overall_status"], preflight.STATUS_UNKNOWN)
        codes = {f["code"] for f in report["checks"]["ros_environment"]["findings"]}
        self.assertIn("PROCESS_INSPECTION_FAILED", codes)

    def test_10_unapproved_vs_approved_dangerous_action(self) -> None:
        unapproved = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-danger-unapproved",
            scope="agent_harness",
            base_commit=self.base_sha,
            requested_actions=["real_limo_connect_or_motor", "git_write_or_pr"],
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(unapproved["overall_status"], preflight.STATUS_BLOCKED)
        self.assertEqual(unapproved["checks"]["dangerous_operations"]["status"], preflight.STATUS_BLOCKED)

        approved = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-danger-approved",
            scope="agent_harness",
            base_commit=self.base_sha,
            requested_actions=["git_write_or_pr"],
            approved_actions=["git_write_or_pr"],
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(approved["overall_status"], preflight.STATUS_WARN)
        self.assertIn("MUST NOT be interpreted as robot motion safety approval", approved["disclaimer"])

    def test_11_nonexistent_and_non_git_path(self) -> None:
        missing = preflight.run_preflight(
            repo_path=self.tmp_path / "does_not_exist",
            agent="Antigravity",
            task_id="t-missing",
        )
        self.assertEqual(missing["overall_status"], preflight.STATUS_BLOCKED)
        self.assertEqual(missing["checks"]["git"]["status"], preflight.STATUS_BLOCKED)
        self.assertEqual(missing["checks"]["scope"]["status"], preflight.STATUS_SKIP)

        not_git = self.tmp_path / "plain_dir"
        not_git.mkdir()
        res = preflight.run_preflight(
            repo_path=not_git,
            agent="Antigravity",
            task_id="t-nongit",
        )
        self.assertEqual(res["overall_status"], preflight.STATUS_BLOCKED)
        self.assertEqual(res["checks"]["git"]["status"], preflight.STATUS_BLOCKED)

    def test_12_cli_json_schema_and_secret_redaction(self) -> None:
        json_out = self.tmp_path / "report.json"
        env = {
            **dict(preflight.os.environ),
            "ROS_DISTRO": "humble",
            "ROS_DOMAIN_ID": "173",
            "ROS_LOCALHOST_ONLY": "1",
            "SUPER_SECRET_TOKEN": "should_never_appear_in_output_98765",
        }
        proc = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().parent / "preflight.py"),
                "--repo",
                str(self.repo),
                "--agent",
                "Antigravity",
                "--task-id",
                "t-cli-json",
                "--scope",
                "agent_harness",
                "--base-commit",
                self.base_sha,
                "--expected-branch",
                "feat/agent-harness-m0-test",
                "--skip-ros",
                "--json",
                "--json-out",
                str(json_out),
            ],
            capture_output=True,
            text=True,
            env=env,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr)
        self.assertNotIn("should_never_appear_in_output_98765", proc.stdout)
        parsed = json.loads(proc.stdout)
        for key in (
            "checked_at",
            "overall_status",
            "agent",
            "task_id",
            "repository",
            "branch",
            "head",
            "declared_scope",
            "checks",
            "recommended_actions",
            "disclaimer",
        ):
            self.assertIn(key, parsed)
        self.assertEqual(parsed, json.loads(json_out.read_text(encoding="utf-8")))

        # Verify cmdline redaction helper directly
        redacted = preflight._redact_cmdline("agent --token secret123 token_file:=/tmp/a.token password=pw")
        self.assertNotIn("secret123", redacted)
        self.assertNotIn("/tmp/a.token", redacted)
        self.assertNotIn("pw", redacted)

    def test_13_existing_worktrees_invariant_before_and_after(self) -> None:
        """Verify running preflight CLI causes zero mutations to existing Capstone1 worktrees."""
        parent = Path("/home/cjinw/school/2026-2")
        targets = [
            parent / "Capstone1_limo",
            parent / "Capstone1_limo_g2_20261123",
            parent / "Capstone1_limo_web_ui_20261123",
            parent / "Capstone1_limo_nav_validation_20261124",
        ]
        existing = [p for p in targets if p.exists()]
        if not existing:
            self.skipTest("Real Capstone1 worktrees not present in this environment")

        def snapshot():
            return {
                str(p): (_git(p, "rev-parse", "HEAD"), _git(p, "status", "--porcelain=v1"))
                for p in existing
            }

        before = snapshot()
        for p in existing:
            subprocess.run(
                [
                    sys.executable,
                    str(Path(__file__).resolve().parent / "preflight.py"),
                    "--repo",
                    str(p),
                    "--agent",
                    "Antigravity",
                    "--task-id",
                    "t-invariance-check",
                    "--scope",
                    "agent_harness",
                    "--json",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
        after = snapshot()
        self.assertEqual(before, after)

    def test_14_pre_existing_cross_worktree_overlap_is_warn_not_blocked(self) -> None:
        sib = self.tmp_path / "sibling_wt_warn"
        _git(self.repo, "worktree", "add", "-b", "feat/sib-warn", str(sib), self.base_sha)
        # Both worktrees have a pre-existing dirty edit on demo.py,
        # while the agent's planned --files is an unrelated new file.
        (self.repo / "scripts" / "agent_harness" / "demo.py").write_text("print('local')\n", encoding="utf-8")
        (sib / "scripts" / "agent_harness" / "demo.py").write_text("print('sib')\n", encoding="utf-8")

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-preexist-overlap",
            scope="agent_harness",
            planned_files=["scripts/agent_harness/new_unrelated.py"],
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_WARN)
        codes = {f["code"] for f in report["checks"]["scope"]["findings"]}
        self.assertIn("PRE_EXISTING_CROSS_WORKTREE_OVERLAP", codes)
        self.assertEqual(report["checks"]["dangerous_operations"]["status"], preflight.STATUS_PASS)

    def test_15_shared_common_doc_warn_vs_sibling_collision_blocked(self) -> None:
        # Normal shared doc edit with no sibling collision -> WARN (not BLOCKED)
        warn_rep = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-shared-doc",
            scope="agent_harness",
            planned_files=["HANDOFF.md"],
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(warn_rep["overall_status"], preflight.STATUS_WARN)
        self.assertIn("SHARED_COMMON_DOC", {f["code"] for f in warn_rep["checks"]["scope"]["findings"]})

        # Same shared doc dirty in a sibling worktree -> BLOCKED
        sib = self.tmp_path / "sibling_doc"
        _git(self.repo, "worktree", "add", "-b", "feat/sib-doc", str(sib), self.base_sha)
        (sib / "HANDOFF.md").write_text("# dirty\n", encoding="utf-8")
        blocked_rep = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-shared-doc-blocked",
            scope="agent_harness",
            planned_files=["HANDOFF.md"],
            base_commit=self.base_sha,
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(blocked_rep["overall_status"], preflight.STATUS_BLOCKED)

    def test_16_ros_domain_uncertain_returns_unknown(self) -> None:
        def opaque_ros_procs():
            return (
                True,
                [{"pid": 88888, "domain_id": None, "port": None, "cmd_preview": "nav2_controller"}],
                None,
            )

        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-domain-uncertain",
            scope="agent_harness",
            base_commit=self.base_sha,
            requested_domain=164,
            env=self.clean_env,
            process_inspector=opaque_ros_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["checks"]["ros_environment"]["status"], preflight.STATUS_UNKNOWN)
        self.assertEqual(report["overall_status"], preflight.STATUS_UNKNOWN)
        codes = {f["code"] for f in report["checks"]["ros_environment"]["findings"]}
        self.assertIn("ROS_DOMAIN_IDENTIFICATION_UNCERTAIN", codes)

    def test_17_reserved_test_port_idle_is_warn_not_blocked(self) -> None:
        report = preflight.run_preflight(
            repo_path=self.repo,
            agent="Antigravity",
            task_id="t-reserved-port-idle",
            scope="agent_harness",
            base_commit=self.base_sha,
            requested_ports=[18310],
            env=self.clean_env,
            process_inspector=_clean_procs,
            port_inspector=_clean_ports,
        )
        self.assertEqual(report["overall_status"], preflight.STATUS_WARN)
        self.assertEqual(report["checks"]["ros_environment"]["status"], preflight.STATUS_WARN)
        codes = {f["code"] for f in report["checks"]["ros_environment"]["findings"]}
        self.assertIn("RESERVED_TEST_PORT_REQUESTED", codes)

    def test_18_malformed_or_directory_config_exits_3(self) -> None:
        bad_cfg = self.tmp_path / "bad.json"
        bad_cfg.write_text("{not valid json", encoding="utf-8")
        p1 = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().parent / "preflight.py"),
                "--repo",
                str(self.repo),
                "--agent",
                "Antigravity",
                "--task-id",
                "t-bad-cfg",
                "--config",
                str(bad_cfg),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(p1.returncode, 3)
        self.assertIn("failed to load config", p1.stderr)

        p2 = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve().parent / "preflight.py"),
                "--repo",
                str(self.repo),
                "--agent",
                "Antigravity",
                "--task-id",
                "t-dir-cfg",
                "--config",
                str(self.tmp_path),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(p2.returncode, 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
