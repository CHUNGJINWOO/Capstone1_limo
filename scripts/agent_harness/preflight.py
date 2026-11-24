#!/usr/bin/env python3
"""Agent Development Harness v0 (M0) — Read-only Development Preflight CLI.

This is a development-hygiene preflight tool for AI coding agents (Claude Code,
Codex, Antigravity, etc.) working in parallel on Capstone1_limo. It is NOT a
ROS 2 runtime test harness and MUST NOT be interpreted as robot motion safety
approval.

Hard guarantees:
- Strictly read-only: never executes git reset/clean/stash/commit/push/merge/
  checkout/worktree remove.
- Never sends signals to processes, never binds ports, never starts ROS nodes.
- Never prints or records secrets, tokens, passwords, or arbitrary env dumps.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

STATUS_PASS = "PASS"
STATUS_WARN = "WARN"
STATUS_BLOCKED = "BLOCKED"
STATUS_UNKNOWN = "UNKNOWN"
STATUS_SKIP = "SKIP"

ALLOWED_ROS_ENV_KEYS = (
    "ROS_DISTRO",
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "IGN_PARTITION",
    "RMW_IMPLEMENTATION",
)

REDACT_PATTERNS = (
    re.compile(r"(token[_-]?file\s*[:=]\s*)(\S+)", re.IGNORECASE),
    re.compile(r"(--?token\s+)(\S+)", re.IGNORECASE),
    re.compile(r"(password\s*[:=]\s*)(\S+)", re.IGNORECASE),
    re.compile(r"(secret\s*[:=]\s*)(\S+)", re.IGNORECASE),
    re.compile(r"(bearer\s+)(\S+)", re.IGNORECASE),
)

DISCLAIMER = (
    "M0 preflight PASS is a read-only development check only and MUST NOT be "
    "interpreted as robot motion safety approval or physical LIMO readiness."
)

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "scopes.json"


def _now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def _redact_cmdline(cmd: str) -> str:
    out = cmd
    for pat in REDACT_PATTERNS:
        out = pat.sub(r"\1<REDACTED>", out)
    return out


def _norm_rel(path_str: str) -> str:
    cleaned = path_str.strip().replace("\\", "/")
    while cleaned.startswith("./"):
        cleaned = cleaned[2:]
    return cleaned.rstrip("/") if cleaned != "/" else cleaned


def _git_readonly(cwd: Path, args: Sequence[str], timeout: float = 5.0) -> Tuple[int, str, str]:
    """Run strictly read-only git subcommands."""
    allowed_subcommands = {"rev-parse", "status", "worktree", "merge-base"}
    if not args or args[0] not in allowed_subcommands:
        raise ValueError(f"Refusing non-allowlisted git subcommand: {args}")
    if args[0] == "worktree" and (len(args) < 2 or args[1] != "list"):
        raise ValueError(f"Refusing non-read-only git worktree subcommand: {args}")
    try:
        proc = subprocess.run(
            ["git", "-C", str(cwd), *args],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout, proc.stderr
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 127, "", str(exc)


def parse_git_status_porcelain(output: str) -> Dict[str, List[str]]:
    """Categorize git status --porcelain=v1 lines into staged, unstaged, both, untracked."""
    staged: List[str] = []
    unstaged: List[str] = []
    staged_and_unstaged: List[str] = []
    untracked: List[str] = []

    for raw_line in output.splitlines():
        if len(raw_line) < 4:
            continue
        x, y = raw_line[0], raw_line[1]
        path_part = raw_line[3:]
        if " -> " in path_part:
            path_part = path_part.split(" -> ", 1)[1]
        path_part = _norm_rel(path_part.strip('"'))
        if x == "?" and y == "?":
            untracked.append(path_part)
        elif x != " " and y != " ":
            staged_and_unstaged.append(path_part)
        elif x != " ":
            staged.append(path_part)
        elif y != " ":
            unstaged.append(path_part)

    all_changed = sorted(set(staged + unstaged + staged_and_unstaged + untracked))
    return {
        "staged": sorted(set(staged)),
        "unstaged": sorted(set(unstaged)),
        "staged_and_unstaged": sorted(set(staged_and_unstaged)),
        "untracked": sorted(set(untracked)),
        "all_changed": all_changed,
    }


def parse_worktree_list_porcelain(output: str) -> List[Dict[str, str]]:
    worktrees: List[Dict[str, str]] = []
    current: Dict[str, str] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line:
            if current:
                worktrees.append(current)
                current = {}
            continue
        if line.startswith("worktree "):
            current["path"] = line[len("worktree "):]
        elif line.startswith("HEAD "):
            current["head"] = line[len("HEAD "):]
        elif line.startswith("branch "):
            branch = line[len("branch "):]
            if branch.startswith("refs/heads/"):
                branch = branch[len("refs/heads/"):]
            current["branch"] = branch
        elif line == "detached":
            current["branch"] = "DETACHED"
        elif line.startswith("prunable"):
            current["prunable"] = "true"
    if current:
        worktrees.append(current)
    return worktrees


def load_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def resolve_scope_key(config: Dict[str, Any], scope_input: Optional[str]) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    if not scope_input:
        return None, None
    scopes = config.get("scopes") or {}
    norm = scope_input.strip().lower()
    if norm in scopes:
        return norm, scopes[norm]
    for key, entry in scopes.items():
        label = str(entry.get("label", "")).lower()
        if norm == label or norm in label:
            return key, entry
    return None, None


def classify_path_owner(rel_path: str, config: Dict[str, Any]) -> Dict[str, Any]:
    norm = _norm_rel(rel_path)
    shared_docs = {_norm_rel(p) for p in config.get("shared_common_docs", [])}
    if norm in shared_docs:
        return {"kind": "SHARED_COMMON_DOC", "scope_key": None, "owner": "SHARED"}

    scopes = config.get("scopes") or {}
    matched: List[Tuple[int, str, Dict[str, Any]]] = []
    for key, entry in scopes.items():
        excluded = False
        for ex in entry.get("exclude_prefixes", []):
            ex_n = _norm_rel(ex)
            if norm == ex_n or norm.startswith(ex_n + "/"):
                excluded = True
                break
        if excluded:
            continue
        for pref in entry.get("path_prefixes", []):
            pref_n = _norm_rel(pref)
            if norm == pref_n or norm.startswith(pref_n + "/"):
                matched.append((len(pref_n), key, entry))

    if matched:
        matched.sort(key=lambda item: item[0], reverse=True)
        _, key, entry = matched[0]
        return {
            "kind": "REGISTERED",
            "scope_key": key,
            "scope_label": entry.get("label", key),
            "owner": entry.get("default_owner", "UNKNOWN"),
        }

    return {"kind": "UNREGISTERED", "scope_key": None, "owner": None}


def default_process_inspector() -> Tuple[bool, List[Dict[str, Any]], Optional[str]]:
    """Read-only inspection of running processes via ps. Never sends signals."""
    try:
        proc = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, [], f"ps invocation failed: {exc}"
    if proc.returncode != 0:
        return False, [], f"ps exited with code {proc.returncode}: {proc.stderr.strip()}"

    ros_keywords = (
        "ros2",
        "rclpy",
        "nav2_",
        "ign gazebo",
        "gz sim",
        "gazebo",
        "fleet_server",
        "limo_robot_agent",
        "motion_watchdog",
        "stall_detector",
        "kinematic_sim",
        "rmw_fastrtps",
        "ros2-daemon",
    )
    items: List[Dict[str, Any]] = []
    domain_re = re.compile(r"(?:--ros-domain-id\s+|ROS_DOMAIN_ID=)(\d+)")
    port_re = re.compile(r"--port\s+(\d+)")

    for line in proc.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split(None, 1)
        if len(parts) != 2 or not parts[0].isdigit():
            continue
        pid = int(parts[0])
        if pid == os.getpid():
            continue
        args = parts[1]
        low = args.lower()
        if not any(kw in low for kw in ros_keywords):
            continue
        dom_match = domain_re.search(args)
        port_match = port_re.search(args)
        items.append(
            {
                "pid": pid,
                "domain_id": int(dom_match.group(1)) if dom_match else None,
                "port": int(port_match.group(1)) if port_match else None,
                "cmd_preview": _redact_cmdline(args)[:180],
            }
        )
    return True, items, None


def default_port_inspector() -> Tuple[bool, List[int], Optional[str]]:
    """Read-only inspection of listening TCP ports via ss. Never binds ports."""
    try:
        proc = subprocess.run(
            ["ss", "-tlnH"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, [], f"ss invocation failed: {exc}"
    if proc.returncode != 0:
        return False, [], f"ss exited with code {proc.returncode}: {proc.stderr.strip()}"

    ports: set[int] = set()
    for line in proc.stdout.splitlines():
        cols = line.split()
        if len(cols) < 4:
            continue
        local_addr = cols[3]
        if ":" in local_addr:
            port_str = local_addr.rsplit(":", 1)[-1]
            if port_str.isdigit():
                ports.add(int(port_str))
    return True, sorted(ports), None


def _check_git(
    repo_path: Path,
    base_commit: Optional[str],
    expected_branch: Optional[str],
    inspect_siblings: bool,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    if not repo_path.exists():
        return (
            {
                "status": STATUS_BLOCKED,
                "error": f"Path does not exist: {repo_path}",
            },
            {},
        )
    if not repo_path.is_dir():
        return (
            {
                "status": STATUS_BLOCKED,
                "error": f"Path is not a directory: {repo_path}",
            },
            {},
        )

    rc, top_out, top_err = _git_readonly(repo_path, ["rev-parse", "--show-toplevel"])
    if rc != 0:
        return (
            {
                "status": STATUS_BLOCKED,
                "error": f"Not a valid Git repository/worktree: {top_err.strip() or repo_path}",
            },
            {},
        )
    toplevel = Path(top_out.strip()).resolve()

    _, head_out, _ = _git_readonly(toplevel, ["rev-parse", "HEAD"])
    head_sha = head_out.strip()
    _, branch_out, _ = _git_readonly(toplevel, ["rev-parse", "--abbrev-ref", "HEAD"])
    branch = branch_out.strip()
    _, wt_out, _ = _git_readonly(toplevel, ["worktree", "list", "--porcelain"])
    worktrees = parse_worktree_list_porcelain(wt_out)
    _, st_out, _ = _git_readonly(toplevel, ["status", "--porcelain=v1"])
    status_summary = parse_git_status_porcelain(st_out)

    findings: List[Dict[str, str]] = []
    section_status = STATUS_PASS

    if expected_branch and branch != expected_branch:
        section_status = STATUS_WARN
        findings.append(
            {
                "level": STATUS_WARN,
                "code": "BRANCH_MISMATCH",
                "message": f"Current branch '{branch}' differs from expected '{expected_branch}'.",
            }
        )

    base_info: Dict[str, Any] = {"requested": base_commit, "head_matches_base": None, "is_ancestor": None}
    if base_commit:
        rc_b, base_sha_out, _ = _git_readonly(toplevel, ["rev-parse", "--verify", f"{base_commit}^{{commit}}"])
        if rc_b != 0:
            section_status = STATUS_WARN
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "BASE_COMMIT_NOT_FOUND",
                    "message": f"Base commit '{base_commit}' could not be resolved in this repository.",
                }
            )
        else:
            base_full = base_sha_out.strip()
            base_info["resolved"] = base_full
            base_info["head_matches_base"] = head_sha == base_full
            rc_anc, _, _ = _git_readonly(toplevel, ["merge-base", "--is-ancestor", base_full, head_sha])
            base_info["is_ancestor"] = rc_anc == 0
            if head_sha != base_full:
                section_status = STATUS_WARN
                relation = "descendant of" if rc_anc == 0 else "diverged from"
                findings.append(
                    {
                        "level": STATUS_WARN,
                        "code": "HEAD_BASE_MISMATCH",
                        "message": f"HEAD {head_sha[:8]} is {relation} base commit {base_full[:8]} (not exact match).",
                    }
                )

    if status_summary["all_changed"]:
        if section_status == STATUS_PASS:
            section_status = STATUS_WARN
        findings.append(
            {
                "level": STATUS_WARN,
                "code": "DIRTY_WORKTREE",
                "message": (
                    f"Worktree has {len(status_summary['all_changed'])} uncommitted/untracked entry(ies) "
                    f"(staged={len(status_summary['staged'])}, unstaged={len(status_summary['unstaged'])}, "
                    f"both={len(status_summary['staged_and_unstaged'])}, untracked={len(status_summary['untracked'])}). "
                    "Read-only preflight continues without failing solely for dirtiness."
                ),
            }
        )

    sibling_dirty: Dict[str, List[str]] = {}
    if inspect_siblings:
        for wt in worktrees:
            wt_path = Path(wt["path"]).resolve()
            if wt_path == toplevel or not wt_path.exists():
                continue
            rc_s, sib_out, _ = _git_readonly(wt_path, ["status", "--porcelain=v1"])
            if rc_s == 0:
                sib_parsed = parse_git_status_porcelain(sib_out)
                if sib_parsed["all_changed"]:
                    sibling_dirty[str(wt_path)] = sib_parsed["all_changed"]

    git_result = {
        "status": section_status,
        "toplevel": str(toplevel),
        "branch": branch,
        "head": head_sha,
        "base_commit": base_info,
        "worktree_count": len(worktrees),
        "worktrees": worktrees,
        "changed_files": status_summary,
        "findings": findings,
    }
    return git_result, sibling_dirty


def _check_scope_and_conflicts(
    agent: str,
    scope_input: Optional[str],
    planned_files: Sequence[str],
    changed_files: Sequence[str],
    sibling_dirty: Dict[str, List[str]],
    config: Dict[str, Any],
) -> Dict[str, Any]:
    scope_key, scope_entry = resolve_scope_key(config, scope_input)
    planned_norm = sorted({_norm_rel(p) for p in planned_files if p.strip()})
    changed_norm = sorted({_norm_rel(p) for p in changed_files if p.strip()})
    planned_set = set(planned_norm)

    pre_existing_outside_planned = [p for p in changed_norm if p not in planned_set]
    overlapping_with_pre_existing = [p for p in planned_norm if p in set(changed_norm)]

    findings: List[Dict[str, Any]] = []
    status = STATUS_PASS

    if scope_input and scope_key is None:
        status = STATUS_WARN
        findings.append(
            {
                "level": STATUS_WARN,
                "code": "UNREGISTERED_SCOPE",
                "message": (
                    f"Declared scope '{scope_input}' is not registered in scopes.json; "
                    "ownership will NOT be auto-inferred."
                ),
            }
        )

    def _bump(new_level: str) -> None:
        nonlocal status
        rank = {STATUS_PASS: 0, STATUS_SKIP: 0, STATUS_WARN: 1, STATUS_UNKNOWN: 1, STATUS_BLOCKED: 2}
        if rank[new_level] > rank[status]:
            status = new_level

    # Evaluate planned files + current dirty files
    file_classifications: List[Dict[str, Any]] = []
    inspected_paths = sorted(set(planned_norm) | set(changed_norm))
    for rel in inspected_paths:
        source = (
            "planned+dirty"
            if (rel in planned_set and rel in set(changed_norm))
            else ("planned" if rel in planned_set else "pre_existing_dirty")
        )
        cls = classify_path_owner(rel, config)
        entry_rec = {"path": rel, "source": source, **cls}
        file_classifications.append(entry_rec)

        if cls["kind"] == "SHARED_COMMON_DOC":
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "SHARED_COMMON_DOC",
                    "path": rel,
                    "source": source,
                    "message": (
                        f"'{rel}' ({source}) is a shared common document; prefer writing a change proposal "
                        "instead of direct concurrent edits."
                    ),
                }
            )
        elif cls["kind"] == "UNREGISTERED":
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "UNREGISTERED_PATH",
                    "path": rel,
                    "source": source,
                    "message": (
                        f"'{rel}' ({source}) does not belong to any registered scope; ownership is UNREGISTERED "
                        "(manual check required)."
                    ),
                }
            )
        elif cls["kind"] == "REGISTERED":
            owner = str(cls.get("owner", ""))
            other_scope = scope_key is not None and cls["scope_key"] != scope_key
            other_agent = bool(agent) and owner.lower() != agent.strip().lower()
            if other_scope or other_agent:
                # Only planned edits into another agent's/scope's area trigger BLOCKED;
                # pre-existing dirty files outside --files are advisory WARN so dirty worktrees aren't falsely blocked.
                lvl = STATUS_BLOCKED if source in ("planned", "planned+dirty") else STATUS_WARN
                _bump(lvl)
                findings.append(
                    {
                        "level": lvl,
                        "code": "OTHER_AGENT_SCOPE_CONFLICT",
                        "path": rel,
                        "source": source,
                        "registered_owner": owner,
                        "registered_scope": cls["scope_key"],
                        "message": (
                            f"'{rel}' ({source}) belongs to scope '{cls['scope_key']}' "
                            f"(default owner: {owner}), conflicting with agent='{agent}', scope='{scope_input}'."
                        ),
                    }
                )

    # Cross-worktree collision detection:
    # - Planned file colliding with another worktree's uncommitted file -> BLOCKED
    # - Pre-existing dirty file overlapping with another worktree -> WARN (advisory, not a hard block on unrelated work)
    cross_wt_conflicts: List[Dict[str, str]] = []
    for sib_wt, sib_files in sibling_dirty.items():
        sib_set = set(sib_files)
        for rel in inspected_paths:
            if rel in sib_set:
                is_planned = rel in planned_set or not planned_set
                lvl = STATUS_BLOCKED if is_planned else STATUS_WARN
                code = (
                    "CROSS_WORKTREE_UNCOMMITTED_COLLISION"
                    if is_planned
                    else "PRE_EXISTING_CROSS_WORKTREE_OVERLAP"
                )
                _bump(lvl)
                conflict = {
                    "path": rel,
                    "sibling_worktree": sib_wt,
                    "level": lvl,
                    "source": "planned" if rel in planned_set else "pre_existing_dirty",
                }
                cross_wt_conflicts.append(conflict)
                findings.append(
                    {
                        "level": lvl,
                        "code": code,
                        "path": rel,
                        "source": conflict["source"],
                        "sibling_worktree": sib_wt,
                        "message": (
                            f"'{rel}' ({conflict['source']}) also has uncommitted/untracked changes in sibling worktree "
                            f"'{sib_wt}'."
                        ),
                    }
                )

    return {
        "status": status,
        "declared_agent": agent,
        "declared_scope_input": scope_input,
        "resolved_scope_key": scope_key,
        "resolved_scope_label": scope_entry.get("label") if scope_entry else None,
        "planned_files": planned_norm,
        "pre_existing_dirty_outside_planned": pre_existing_outside_planned,
        "planned_overlapping_pre_existing_dirty": overlapping_with_pre_existing,
        "file_classifications": file_classifications,
        "cross_worktree_conflicts": cross_wt_conflicts,
        "tracking_limitation": (
            "M0 preflight only inspects --files and current git status before work starts; "
            "unplanned edits made afterward are not tracked until re-running preflight or M1 postflight."
        ),
        "findings": findings,
    }


def _check_ros_env(
    requested_domain: Optional[int],
    requested_ports: Sequence[int],
    config: Dict[str, Any],
    env: Dict[str, str],
    process_inspector: Callable[[], Tuple[bool, List[Dict[str, Any]], Optional[str]]],
    port_inspector: Callable[[], Tuple[bool, List[int], Optional[str]]],
) -> Dict[str, Any]:
    ros_env: Dict[str, Optional[str]] = {k: env.get(k) for k in ALLOWED_ROS_ENV_KEYS}
    effective_domain: Optional[int] = requested_domain
    if effective_domain is None and ros_env["ROS_DOMAIN_ID"] and ros_env["ROS_DOMAIN_ID"].isdigit():
        effective_domain = int(ros_env["ROS_DOMAIN_ID"])

    findings: List[Dict[str, Any]] = []
    status = STATUS_PASS

    def _bump(new_level: str) -> None:
        nonlocal status
        rank = {STATUS_PASS: 0, STATUS_SKIP: 0, STATUS_WARN: 1, STATUS_UNKNOWN: 2, STATUS_BLOCKED: 3}
        if rank[new_level] > rank[status]:
            status = new_level

    proc_ok, ros_procs, proc_err = process_inspector()
    port_ok, listening_ports, port_err = port_inspector()

    if not proc_ok:
        _bump(STATUS_UNKNOWN)
        findings.append(
            {
                "level": STATUS_UNKNOWN,
                "code": "PROCESS_INSPECTION_FAILED",
                "message": f"Could not inspect running ROS processes: {proc_err}",
            }
        )

    if not port_ok:
        _bump(STATUS_UNKNOWN)
        findings.append(
            {
                "level": STATUS_UNKNOWN,
                "code": "PORT_INSPECTION_FAILED",
                "message": f"Could not inspect listening ports: {port_err}",
            }
        )

    active_domains = sorted({p["domain_id"] for p in ros_procs if p.get("domain_id") is not None})
    unknown_domain_procs = [p for p in ros_procs if p.get("domain_id") is None]
    reserved_domains = {int(k): v for k, v in (config.get("reserved_ros_domains") or {}).items()}
    protected_ports = {int(k): v for k, v in (config.get("protected_ports") or {}).items()}
    reserved_test_ports = {int(k): v for k, v in (config.get("reserved_test_ports") or {}).items()}

    if effective_domain is not None:
        if effective_domain in active_domains:
            _bump(STATUS_BLOCKED)
            findings.append(
                {
                    "level": STATUS_BLOCKED,
                    "code": "ROS_DOMAIN_COLLISION_ACTIVE",
                    "domain_id": effective_domain,
                    "message": (
                        f"ROS_DOMAIN_ID={effective_domain} is already in use by running process(es). "
                        "Select an unused isolated domain."
                    ),
                }
            )
        elif proc_ok and unknown_domain_procs:
            _bump(STATUS_UNKNOWN)
            findings.append(
                {
                    "level": STATUS_UNKNOWN,
                    "code": "ROS_DOMAIN_IDENTIFICATION_UNCERTAIN",
                    "domain_id": effective_domain,
                    "unknown_domain_process_count": len(unknown_domain_procs),
                    "message": (
                        f"{len(unknown_domain_procs)} active ROS/Fleet/Gazebo process(es) do not expose "
                        f"ROS_DOMAIN_ID in command arguments; cannot prove domain {effective_domain} is unused."
                    ),
                }
            )
        if effective_domain in reserved_domains:
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "ROS_DOMAIN_RESERVED",
                    "domain_id": effective_domain,
                    "message": (
                        f"ROS_DOMAIN_ID={effective_domain} is reserved in WORKTREES.md: "
                        f"{reserved_domains[effective_domain]}."
                    ),
                }
            )
        if ros_env["ROS_LOCALHOST_ONLY"] != "1":
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "ROS_LOCALHOST_ONLY_NOT_SET",
                    "message": (
                        f"ROS_LOCALHOST_ONLY is '{ros_env['ROS_LOCALHOST_ONLY']}' (recommend '1' "
                        "for isolated local development)."
                    ),
                }
            )

    active_protected_ports = [
        {"port": p, "purpose": protected_ports[p]}
        for p in listening_ports
        if p in protected_ports
    ]
    for port in requested_ports:
        if port in protected_ports:
            _bump(STATUS_BLOCKED)
            findings.append(
                {
                    "level": STATUS_BLOCKED,
                    "code": "PROTECTED_PORT_REQUESTED",
                    "port": port,
                    "message": f"Requested port {port} is protected ({protected_ports[port]}).",
                }
            )
        elif port_ok and port in listening_ports:
            _bump(STATUS_BLOCKED)
            findings.append(
                {
                    "level": STATUS_BLOCKED,
                    "code": "PORT_ALREADY_IN_USE",
                    "port": port,
                    "message": f"Requested test port {port} is already listening.",
                }
            )
        elif port in reserved_test_ports:
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "RESERVED_TEST_PORT_REQUESTED",
                    "port": port,
                    "message": (
                        f"Requested port {port} is allocated in WORKTREES.md ({reserved_test_ports[port]}) "
                        "but is not currently listening; confirm no parallel session expects it."
                    ),
                }
            )

    if ros_procs and effective_domain is None:
        if unknown_domain_procs:
            _bump(STATUS_UNKNOWN)
            findings.append(
                {
                    "level": STATUS_UNKNOWN,
                    "code": "ROS_DOMAIN_IDENTIFICATION_UNCERTAIN",
                    "unknown_domain_process_count": len(unknown_domain_procs),
                    "message": (
                        f"Detected {len(ros_procs)} active ROS/Fleet/Gazebo process(es) "
                        f"({len(unknown_domain_procs)} without explicit --ros-domain-id in args; "
                        f"parsed domains={active_domains}). Pass --domain and --ports before running ROS tests."
                    ),
                }
            )
        else:
            _bump(STATUS_WARN)
            findings.append(
                {
                    "level": STATUS_WARN,
                    "code": "ACTIVE_ROS_PROCESSES_DETECTED",
                    "message": (
                        f"Detected {len(ros_procs)} active ROS/Fleet/Gazebo process(es) "
                        f"(active domains: {active_domains}). "
                        "Pass --domain and --ports if you plan to run isolated tests."
                    ),
                }
            )

    return {
        "status": status,
        "env": ros_env,
        "effective_domain_id": effective_domain,
        "process_inspection_ok": proc_ok,
        "ros_process_count": len(ros_procs) if proc_ok else None,
        "unknown_domain_process_count": len(unknown_domain_procs) if proc_ok else None,
        "ros_processes_sample": ros_procs[:15] if proc_ok else [],
        "active_domains_detected": active_domains,
        "port_inspection_ok": port_ok,
        "listening_protected_ports": active_protected_ports,
        "requested_ports": list(requested_ports),
        "findings": findings,
    }


def _check_dangerous_operations(
    requested_actions: Sequence[str],
    approved_actions: Sequence[str],
    planned_files: Sequence[str],
    changed_files: Sequence[str],
    requested_ports: Sequence[int],
    cross_wt_conflicts: Sequence[Dict[str, str]],
    config: Dict[str, Any],
) -> Dict[str, Any]:
    catalog = config.get("dangerous_actions") or {}
    approved_set = {a.strip() for a in approved_actions if a.strip()}
    detected: Dict[str, List[str]] = {}

    def _add(action_key: str, reason: str) -> None:
        detected.setdefault(action_key, []).append(reason)

    for act in requested_actions:
        key = act.strip()
        if not key:
            continue
        _add(key, "explicitly requested via --actions")

    if 18190 in requested_ports or 8080 in requested_ports:
        _add("prod_server_18190", f"requested protected server port in {list(requested_ports)}")

    # When --files is explicitly provided, evaluate danger against planned_files so pre-existing
    # dirty files in another area do not falsely BLOCK an unrelated read-only/narrow task.
    planned_norm = sorted({_norm_rel(p) for p in planned_files if p.strip()})
    inspected = planned_norm if planned_norm else sorted({_norm_rel(p) for p in changed_files if p.strip()})
    safety_paths = {_norm_rel(p) for p in config.get("safety_contract_paths", [])}
    exp_prefixes = [_norm_rel(p) for p in config.get("protected_experiment_prefixes", [])]

    for rel in inspected:
        if rel in safety_paths:
            _add("safety_contract_change", f"touches safety contract path '{rel}'")
        for pref in exp_prefixes:
            if rel == pref or rel.startswith(pref + "/"):
                _add("delete_experiment_originals", f"touches protected experiment/data path '{rel}'")

    blocking_cross = [c for c in cross_wt_conflicts if c.get("level", STATUS_BLOCKED) == STATUS_BLOCKED]
    if blocking_cross:
        paths = ", ".join(sorted({c["path"] for c in blocking_cross}))
        _add("overwrite_other_agent_uncommitted", f"cross-worktree uncommitted collision on: {paths}")

    findings: List[Dict[str, Any]] = []
    status = STATUS_PASS
    for key, reasons in sorted(detected.items()):
        meta = catalog.get(key, {"label": key, "description": "Unregistered dangerous action"})
        if key in approved_set:
            if status == STATUS_PASS:
                status = STATUS_WARN
            findings.append(
                {
                    "level": STATUS_WARN,
                    "action": key,
                    "approved": True,
                    "label": meta.get("label", key),
                    "reasons": reasons,
                    "message": (
                        f"Dangerous action '{key}' ({meta.get('label', key)}) is marked approved; "
                        "human judgment and physical safety procedures still apply."
                    ),
                }
            )
        else:
            status = STATUS_BLOCKED
            findings.append(
                {
                    "level": STATUS_BLOCKED,
                    "action": key,
                    "approved": False,
                    "label": meta.get("label", key),
                    "reasons": reasons,
                    "message": (
                        f"Unapproved dangerous action '{key}' ({meta.get('label', key)}): "
                        f"{meta.get('description', '')}. Explicit human approval required."
                    ),
                }
            )

    return {
        "status": status,
        "requested_actions": sorted({a.strip() for a in requested_actions if a.strip()}),
        "approved_actions": sorted(approved_set),
        "detected_count": len(detected),
        "findings": findings,
    }


def _aggregate_overall_status(statuses: Iterable[str]) -> str:
    items = list(statuses)
    if STATUS_BLOCKED in items:
        return STATUS_BLOCKED
    if STATUS_UNKNOWN in items:
        return STATUS_UNKNOWN
    if STATUS_WARN in items:
        return STATUS_WARN
    if items and all(s == STATUS_SKIP for s in items):
        return STATUS_SKIP
    return STATUS_PASS


def _build_recommendations(report: Dict[str, Any]) -> List[str]:
    recs: List[str] = []
    git_sec = report["checks"]["git"]
    scope_sec = report["checks"]["scope"]
    ros_sec = report["checks"]["ros_environment"]
    danger_sec = report["checks"]["dangerous_operations"]

    if git_sec["status"] == STATUS_BLOCKED:
        recs.append("Verify --repo points to a valid local Git repository or worktree.")
        return recs

    for f in git_sec.get("findings", []):
        if f["code"] == "DIRTY_WORKTREE":
            recs.append("Preserve pre-existing uncommitted files; do not run git reset/clean/stash.")
        elif f["code"] in ("BRANCH_MISMATCH", "HEAD_BASE_MISMATCH"):
            recs.append("Confirm your branch and base commit match the task instructions before editing.")

    for f in scope_sec.get("findings", []):
        if f["code"] == "OTHER_AGENT_SCOPE_CONFLICT" and f.get("level") == STATUS_BLOCKED:
            recs.append(
                f"Do not edit '{f['path']}' directly (owned by {f.get('registered_owner')}); "
                "write a patch proposal document instead."
            )
        elif f["code"] == "CROSS_WORKTREE_UNCOMMITTED_COLLISION":
            recs.append(
                f"Avoid modifying '{f['path']}'; it is already dirty in '{f['sibling_worktree']}'."
            )
        elif f["code"] == "SHARED_COMMON_DOC" and f.get("source") in ("planned", "planned+dirty"):
            recs.append(
                f"Avoid direct concurrent edits to shared doc '{f['path']}'; record a proposed update."
            )
        elif f["code"] == "UNREGISTERED_PATH" and f.get("source") in ("planned", "planned+dirty"):
            recs.append(f"Manually verify ownership of unregistered path '{f['path']}'.")

    for f in ros_sec.get("findings", []):
        if f["code"] == "ROS_DOMAIN_COLLISION_ACTIVE":
            recs.append(f"Choose a different isolated ROS_DOMAIN_ID (domain {f['domain_id']} is active).")
        elif f["code"] == "ROS_DOMAIN_IDENTIFICATION_UNCERTAIN":
            recs.append("Some active ROS processes do not expose ROS_DOMAIN_ID in argv; verify isolation before ROS runs.")
        elif f["code"] in ("PROTECTED_PORT_REQUESTED", "PORT_ALREADY_IN_USE"):
            recs.append(f"Choose a different free test port (port {f['port']} cannot be used).")
        elif f["code"] in ("PROCESS_INSPECTION_FAILED", "PORT_INSPECTION_FAILED"):
            recs.append("Manually verify running ROS processes and open ports before launching tests.")

    for f in danger_sec.get("findings", []):
        if not f.get("approved"):
            recs.append(f"Obtain explicit human approval before performing '{f['action']}'.")

    if not recs:
        recs.append("Preflight checks passed. Proceed within your declared worktree and scope.")
    return list(dict.fromkeys(recs))


def run_preflight(
    repo_path: Path,
    agent: str,
    task_id: str,
    scope: Optional[str] = None,
    planned_files: Sequence[str] = (),
    base_commit: Optional[str] = None,
    expected_branch: Optional[str] = None,
    requested_domain: Optional[int] = None,
    requested_ports: Sequence[int] = (),
    requested_actions: Sequence[str] = (),
    approved_actions: Sequence[str] = (),
    config_path: Path = DEFAULT_CONFIG_PATH,
    inspect_siblings: bool = True,
    skip_ros: bool = False,
    env: Optional[Dict[str, str]] = None,
    process_inspector: Callable[[], Tuple[bool, List[Dict[str, Any]], Optional[str]]] = default_process_inspector,
    port_inspector: Callable[[], Tuple[bool, List[int], Optional[str]]] = default_port_inspector,
) -> Dict[str, Any]:
    config = load_config(config_path)
    effective_base = base_commit if base_commit is not None else config.get("baseline_commit")
    active_env = dict(os.environ if env is None else env)

    git_sec, sibling_dirty = _check_git(
        repo_path=repo_path,
        base_commit=effective_base,
        expected_branch=expected_branch,
        inspect_siblings=inspect_siblings,
    )

    if git_sec["status"] == STATUS_BLOCKED:
        scope_sec = {"status": STATUS_SKIP, "reason": "Git check blocked; scope check skipped.", "findings": []}
        ros_sec = {"status": STATUS_SKIP, "reason": "Git check blocked; ROS check skipped.", "findings": []}
        danger_sec = {"status": STATUS_SKIP, "reason": "Git check blocked; danger check skipped.", "findings": []}
    else:
        changed = git_sec["changed_files"]["all_changed"]
        scope_sec = _check_scope_and_conflicts(
            agent=agent,
            scope_input=scope,
            planned_files=planned_files,
            changed_files=changed,
            sibling_dirty=sibling_dirty,
            config=config,
        )
        if skip_ros:
            ros_sec = {
                "status": STATUS_SKIP,
                "reason": "Skipped by --skip-ros flag.",
                "env": {k: active_env.get(k) for k in ALLOWED_ROS_ENV_KEYS},
                "findings": [],
            }
        else:
            ros_sec = _check_ros_env(
                requested_domain=requested_domain,
                requested_ports=requested_ports,
                config=config,
                env=active_env,
                process_inspector=process_inspector,
                port_inspector=port_inspector,
            )
        danger_sec = _check_dangerous_operations(
            requested_actions=requested_actions,
            approved_actions=approved_actions,
            planned_files=planned_files,
            changed_files=changed,
            requested_ports=requested_ports,
            cross_wt_conflicts=scope_sec.get("cross_worktree_conflicts", []),
            config=config,
        )

    overall = _aggregate_overall_status(
        [git_sec["status"], scope_sec["status"], ros_sec["status"], danger_sec["status"]]
    )

    report: Dict[str, Any] = {
        "harness_version": "0.1.0-m0",
        "checked_at": _now_iso(),
        "overall_status": overall,
        "agent": agent,
        "task_id": task_id,
        "repository": git_sec.get("toplevel", str(repo_path)),
        "branch": git_sec.get("branch"),
        "head": git_sec.get("head"),
        "declared_scope": scope,
        "checks": {
            "git": git_sec,
            "scope": scope_sec,
            "ros_environment": ros_sec,
            "dangerous_operations": danger_sec,
        },
        "recommended_actions": [],
        "disclaimer": DISCLAIMER,
    }
    report["recommended_actions"] = _build_recommendations(report)
    return report


def format_terminal_report(report: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append("=" * 72)
    lines.append(f"Agent Development Harness v0 (M0) — Read-only Preflight [{report['overall_status']}]")
    lines.append("=" * 72)
    lines.append(f"Checked At : {report['checked_at']}")
    lines.append(f"Agent / ID : {report['agent']} / {report['task_id']}")
    lines.append(f"Worktree   : {report['repository']}")
    lines.append(f"Branch/HEAD: {report.get('branch') or 'N/A'} @ {(report.get('head') or 'N/A')[:12]}")
    lines.append(f"Scope      : {report.get('declared_scope') or '(not specified)'}")
    lines.append("-" * 72)

    checks = report["checks"]
    git_sec = checks["git"]
    lines.append(f"[A] Git State        : {git_sec['status']}")
    if "error" in git_sec:
        lines.append(f"    ERROR: {git_sec['error']}")
    else:
        cf = git_sec["changed_files"]
        lines.append(
            f"    Worktrees={git_sec['worktree_count']} | "
            f"staged={len(cf['staged'])}, unstaged={len(cf['unstaged'])}, "
            f"both={len(cf['staged_and_unstaged'])}, untracked={len(cf['untracked'])}"
        )
        for f in git_sec.get("findings", []):
            lines.append(f"    - [{f['level']}] {f['code']}: {f['message']}")

    def _append_capped(items: Sequence[str], limit: int = 15, indent: str = "    ") -> None:
        for line in items[:limit]:
            lines.append(line)
        if len(items) > limit:
            lines.append(f"{indent}... (+{len(items) - limit} more; use --json for full list)")

    scope_sec = checks["scope"]
    lines.append(f"[B] Scope & Conflicts: {scope_sec['status']}")
    if scope_sec.get("resolved_scope_key"):
        lines.append(
            f"    Resolved: {scope_sec['resolved_scope_key']} ({scope_sec.get('resolved_scope_label')})"
        )
    _append_capped(
        [f"    - [{f['level']}] {f['code']}: {f['message']}" for f in scope_sec.get("findings", [])]
    )

    ros_sec = checks["ros_environment"]
    lines.append(f"[C] ROS 2 Environment: {ros_sec['status']}")
    env_map = ros_sec.get("env") or {}
    lines.append(
        f"    ROS_DISTRO={env_map.get('ROS_DISTRO')} | "
        f"ROS_DOMAIN_ID={env_map.get('ROS_DOMAIN_ID')} | "
        f"ROS_LOCALHOST_ONLY={env_map.get('ROS_LOCALHOST_ONLY')}"
    )
    if ros_sec.get("process_inspection_ok"):
        lines.append(
            f"    Active ROS/Fleet/Gazebo procs={ros_sec.get('ros_process_count')} "
            f"(domain_unknown={ros_sec.get('unknown_domain_process_count')}) | "
            f"detected domains={ros_sec.get('active_domains_detected')}"
        )
    _append_capped(
        [f"    - [{f['level']}] {f['code']}: {f['message']}" for f in ros_sec.get("findings", [])]
    )

    danger_sec = checks["dangerous_operations"]
    lines.append(f"[D] Dangerous Actions: {danger_sec['status']}")
    _append_capped(
        [f"    - [{f['level']}] {f['action']}: {f['message']}" for f in danger_sec.get("findings", [])]
    )

    lines.append("-" * 72)
    lines.append("Recommended Next Steps:")
    _append_capped([f"  * {rec}" for rec in report.get("recommended_actions", [])], indent="  ")
    lines.append("-" * 72)
    lines.append(f"NOTE: {report['disclaimer']}")
    return "\n".join(lines)


def _parse_csv_or_multi(values: Optional[Sequence[str]]) -> List[str]:
    out: List[str] = []
    for item in values or ():
        for part in item.split(","):
            part = part.strip()
            if part:
                out.append(part)
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Capstone1 Agent Development Harness v0 (M0) — Read-only Preflight CLI"
    )
    p.add_argument("--repo", default=".", help="Path to target repository or worktree (default: .)")
    p.add_argument("--agent", required=True, help="Agent name (e.g. 'Claude Code', 'Antigravity', 'Codex')")
    p.add_argument("--task-id", required=True, help="Task identifier (e.g. 'harness-m0-20261124')")
    p.add_argument("--scope", default=None, help="Declared scope key or label from scopes.json")
    p.add_argument(
        "--files",
        action="append",
        default=[],
        help="Planned file paths relative to repo root (repeatable or comma-separated)",
    )
    p.add_argument("--base-commit", default=None, help="Expected baseline commit SHA (default from scopes.json)")
    p.add_argument("--expected-branch", default=None, help="Expected Git branch name")
    p.add_argument("--domain", type=int, default=None, help="Planned ROS_DOMAIN_ID for isolated tests")
    p.add_argument(
        "--ports",
        action="append",
        default=[],
        help="Planned TCP test ports (repeatable or comma-separated)",
    )
    p.add_argument(
        "--actions",
        action="append",
        default=[],
        help="Planned dangerous action keys (repeatable or comma-separated)",
    )
    p.add_argument(
        "--approved-actions",
        action="append",
        default=[],
        help="Dangerous action keys explicitly approved by a human",
    )
    p.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="Path to scopes.json")
    p.add_argument("--no-siblings", action="store_true", help="Do not inspect sibling worktrees")
    p.add_argument("--skip-ros", action="store_true", help="Skip ROS 2 environment inspection")
    p.add_argument("--json", dest="as_json", action="store_true", help="Print JSON output to stdout")
    p.add_argument("--json-out", default=None, help="Also write JSON report to this file path")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    port_strs = _parse_csv_or_multi(args.ports)
    ports: List[int] = []
    for ps in port_strs:
        if not ps.isdigit():
            print(f"error: invalid port '{ps}'", file=sys.stderr)
            return 3
        ports.append(int(ps))

    config_path = Path(args.config)
    if not config_path.exists() or not config_path.is_file():
        print(f"error: config file not found or not a regular file: {config_path}", file=sys.stderr)
        return 3

    try:
        report = run_preflight(
            repo_path=Path(args.repo),
            agent=args.agent,
            task_id=args.task_id,
            scope=args.scope,
            planned_files=_parse_csv_or_multi(args.files),
            base_commit=args.base_commit,
            expected_branch=args.expected_branch,
            requested_domain=args.domain,
            requested_ports=ports,
            requested_actions=_parse_csv_or_multi(args.actions),
            approved_actions=_parse_csv_or_multi(args.approved_actions),
            config_path=config_path,
            inspect_siblings=not args.no_siblings,
            skip_ros=args.skip_ros,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"error: failed to load config '{config_path}': {exc}", file=sys.stderr)
        return 3

    json_text = json.dumps(report, indent=2, ensure_ascii=False)
    if args.json_out:
        out_path = Path(args.json_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json_text + "\n", encoding="utf-8")

    if args.as_json:
        print(json_text)
    else:
        print(format_terminal_report(report))

    if report["overall_status"] == STATUS_BLOCKED:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
