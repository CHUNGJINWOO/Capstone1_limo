# AGENTS.md

## 1. Project Overview

This repository is the development repository for the 2026-2 Capstone Design project:

**"LIMO를 이용한 실내 자율 순찰 및 안내"**

The project uses an **AgileX LIMO Pro** mobile robot and ROS 2 to develop an indoor autonomous patrol and guidance system.

The primary objectives are:

- Indoor autonomous navigation
- Autonomous patrol
- Destination guidance
- Static obstacle handling
- Dynamic obstacle avoidance
- Navigation and path-following
- Mapping and localization
- Integration and validation on the actual robot

This repository contains the project's ROS 2 source code, configuration, documentation, experiments, and development records.

---

## 2. Development Scope

The main development areas are:

- LiDAR-based perception
- SLAM and map creation
- Localization / AMCL
- Navigation2 (Nav2)
- Global and local planning
- Costmap configuration
- Path following
- Static obstacle handling
- Dynamic obstacle avoidance
- Waypoint-based autonomous patrol
- ROS 2 Action-based patrol logic
- Destination guidance
- Parameter tuning
- Autonomous navigation validation

Development should proceed incrementally rather than attempting to integrate all functions at once.

---

## 3. Repository Structure

The main ROS 2 source is located under:

```text
src/
└── limo_ros2/
    ├── limo_base/
    ├── limo_car/
    ├── limo_description/
    └── limo_msgs/
```

### `limo_base`

Contains basic LIMO communication, driver, serial communication, TF-related functionality, and launch files.

Important components include:

- LIMO driver
- Serial port
- LIMO protocol
- Basic LIMO node
- TF publishing
- LiDAR-related launch files

### `limo_car`

Contains LIMO robot models and simulation-related resources.

Includes:

- Ackermann models
- Xacro files
- Gazebo resources
- Sensor models
- RViz configurations
- Robot meshes
- Simulation and model launch files

### `limo_description`

Contains robot description and visualization resources.

Includes:

- URDF / Xacro
- Robot meshes
- Gazebo description files
- RViz configurations
- Model visualization launch files

### `limo_msgs`

Contains custom ROS 2 message definitions used by the LIMO system.

---

## 4. Development Principles

### 4.1 Inspect Before Modifying

Before changing code or configuration:

1. Inspect the existing implementation.
2. Check related configuration and launch files.
3. Check relevant documentation.
4. Understand how the existing components are connected.
5. Make the smallest reasonable change.

Do not modify files simply because another implementation appears cleaner.

---

### 4.2 Preserve Existing Structure

The existing project structure should be preserved unless there is a clear technical reason to change it.

Avoid:

- Unnecessary package restructuring
- Unnecessary file renaming
- Replacing working components without justification
- Large-scale refactoring unrelated to the current task
- Introducing duplicate implementations

When structural changes are necessary, explain why they are needed and what components are affected.

---

### 4.3 Work Incrementally

Implement and verify functionality in small steps.

A typical development order is:

```text
Existing LIMO ROS 2 source
        ↓
Sensor / TF verification
        ↓
SLAM / Mapping
        ↓
Localization
        ↓
Navigation2
        ↓
Path following
        ↓
Obstacle handling
        ↓
Dynamic obstacle avoidance
        ↓
Autonomous patrol
        ↓
Destination guidance
        ↓
Integrated validation
```

Do not assume that later stages work merely because earlier stages are complete.

---

## 5. Verification Rules

### 5.1 Build and Test After Changes

After modifying ROS 2 source code, configuration, launch files, or package metadata:

- Build the affected packages.
- Run appropriate tests when available.
- Check for warnings and errors.
- Verify runtime behavior when possible.

Do not claim that a feature works unless it has actually been verified.

---

### 5.2 Distinguish Between Code, Configuration, and Verified Behavior

When describing the project, clearly distinguish:

- What exists in the source code
- What is configured but not yet verified
- What has been tested
- What works only in simulation
- What has been verified on the physical LIMO
- What is planned but not implemented

Avoid describing planned functionality as implemented functionality.

---

### 5.3 Do Not Invent Test Results

Never fabricate:

- Build results
- Runtime results
- Navigation results
- Sensor results
- SLAM results
- Localization results
- Obstacle avoidance results
- Hardware validation results
- Performance measurements

If verification has not been performed, state that it has not been verified.

---

## 6. ROS 2 Development Guidelines

When modifying ROS 2 components:

- Check `package.xml` and `CMakeLists.txt` before changing dependencies.
- Check existing node names, topics, services, actions, parameters, and TF frames before introducing new ones.
- Preserve existing interfaces unless the task explicitly requires an interface change.
- Check launch files and parameter files related to modified nodes.
- Consider TF relationships when changing sensor, localization, or navigation components.
- Avoid introducing duplicate publishers, subscribers, TF broadcasters, or navigation components.
- Prefer existing ROS 2 and Nav2 mechanisms when they already satisfy the requirement.

For navigation-related changes, consider the relationship between:

```text
Sensor Data
    ↓
TF
    ↓
SLAM / Localization
    ↓
Map / Costmap
    ↓
Global Planner
    ↓
Local Controller
    ↓
Velocity Command
    ↓
LIMO
```

---

## 7. Navigation and Autonomous Driving

The project's autonomous navigation stack should be treated as an integrated system.

When working on navigation, consider:

- Sensor availability
- TF correctness
- Map quality
- Localization accuracy
- Costmap configuration
- Global planning
- Local planning / control
- Robot footprint
- Obstacle inflation
- Velocity limits
- Recovery behavior
- Goal tolerance
- Dynamic obstacle response

A navigation problem should not automatically be assumed to be a planner problem. First determine whether the problem originates from sensing, TF, localization, costmaps, planning, control, or the robot interface.

---

## 8. Dynamic Obstacle Avoidance

Dynamic obstacle avoidance is a major development area of this project.

When implementing or tuning dynamic obstacle handling:

1. Verify that obstacle observations are actually being received.
2. Check the relevant sensor data and TF.
3. Check whether the obstacle appears correctly in the appropriate costmap.
4. Determine whether the planner/controller reacts to the obstacle.
5. Verify the resulting robot behavior.
6. Tune parameters only after identifying the relevant component.

Do not solve a sensor, TF, or costmap problem by unnecessarily replacing the navigation planner.

---

## 9. Documentation

Important development decisions, troubleshooting, and verified results should be documented.

Documentation should clearly distinguish:

- Problem
- Cause
- Investigation
- Solution
- Verification
- Remaining limitations

When a significant configuration or implementation decision is made, record the reason for the decision.

Avoid documenting assumptions as established facts.

---

## 10. Git Rules

The repository uses Git for source and development history.

Before making a commit:

```bash
git status
git diff
```

Review the changes and make sure unrelated files are not included.

Commits should:

- Represent a coherent change
- Use clear commit messages
- Avoid unrelated modifications
- Avoid committing generated build artifacts unless explicitly required

Do not rewrite or delete existing project history unless explicitly requested.

---

## 11. Change Management

For every task:

1. Identify the relevant files.
2. Inspect the current implementation.
3. Determine the smallest appropriate change.
4. Make the change.
5. Build or test the affected functionality.
6. Review the resulting diff.
7. Document important findings when necessary.

If a requested change would affect multiple subsystems, explain the dependencies before making broad changes.

---

## 12. Troubleshooting Approach

When something does not work, use the following order:

```text
1. Reproduce the problem
2. Collect the actual error/output
3. Identify the affected component
4. Inspect the relevant source/configuration
5. Form a hypothesis
6. Make the smallest testable change
7. Rebuild / rerun
8. Compare the result
9. Document the cause and solution
```

Do not make multiple unrelated changes at once when diagnosing a problem.

This makes it possible to identify which change actually solved the problem.

---

## 13. Source and Documentation Priority

When investigating project-specific behavior, use the following priority:

1. Project documentation provided by the instructor
2. AgileX official LIMO documentation
3. ROS 2 official documentation
4. Navigation2 official documentation
5. SLAM Toolbox official documentation
6. Reliable community resources

When external information conflicts with the actual project source code, inspect the source and configuration before making a conclusion.

---

## 14. AI Assistant Behavior

AI agents working on this repository should:

- Inspect relevant files before proposing changes.
- Prefer evidence from the repository over assumptions.
- Keep explanations concise but technically accurate.
- Explain **what** is being changed and **why**.
- Mention alternative approaches when they are technically relevant.
- Explain why the selected approach fits the current project.
- Avoid unnecessary refactoring.
- Avoid claiming unverified functionality.
- Preserve working code unless there is a clear reason to change it.
- Ask for clarification when the requested change conflicts with the existing project structure or requirements.

When reporting results, clearly separate:

```text
Verified
Not Verified
Planned
Assumed / Needs Confirmation
```

---

## 15. Environment Independence

The repository must not assume a single development machine or fixed workspace path.

Development may occur on different computers, robot hardware, virtual machines, containers, or other compatible environments.

Therefore:

- Do not hard-code a specific workspace path.
- Do not assume a specific computer is the development machine.
- Do not treat a cloud server as the project's canonical environment.
- Do not assume a particular network configuration unless required by the task.
- Before running environment-dependent commands, inspect the current environment.
- Prefer portable ROS 2 commands and project-relative paths.

Environment-specific information should only be documented when it is required to reproduce a specific issue or configuration.

---

## 16. Repository Safety

Do not:

- Delete project history without a clear reason.
- Replace working architecture unnecessarily.
- Modify unrelated files.
- Modify credentials, SSH keys, secrets, or `.env` files.
- Commit generated build, install, or log directories.

Before destructive or broad changes, confirm the scope of the change.

When a change may affect multiple subsystems, inspect the dependencies first and prefer the smallest change that can be verified.


## 17. Privacy and Publication

Follow [the repository publication policy](docs/PUBLICATION_POLICY.md) before staging or publishing files. School reports, team plans, slide decks and their drafting/review records stay local, including final submissions. Preserve technical and simulation investigation records without publishing student IDs, private contact details, per-person evaluations or credentials. Review document metadata and commit author/committer emails as well as file contents. Use a verified GitHub noreply email where author privacy is required. Never push local history backups or merge retired history after an authorized privacy reset.

For experiment backup, encryption, and Google Drive archival, follow [VALIDATION.md](VALIDATION.md) §2.5. Never mark unexecuted or skipped checks (such as omitted remote decryption) as `PASS` or `true`; record actual verification states (e.g., `skipped_by_user`).

---

## 18. Multi-Agent Coordination and Common Context Documents

1. **Entry Point and Selective Reading**:
   - Start at [START_HERE.md](START_HERE.md) and selectively read only the common context documents relevant to your current task (do not read all common documents unconditionally on every task).
   - Section 3 above reflects the initial `src/limo_ros2` layout; when your task involves locating ROS 2 packages (`capstone_ws/src/`), `fleet_control/`, or experimental bundles, check [CODE_INDEX.md](CODE_INDEX.md).
2. **Worktree and Folder Creation Rules**:
   - Reuse existing directories first before creating any new folder (`capstone_ws/` for ROS 2 code, `fleet_control/` for fleet/RMF/dashboard, `experiments/` with a unique `run_id` for experiments, and approved `local_data/` paths for git-excluded large data and backups).
   - Do not create new git worktrees or new project top-level directories without explicit user approval.
   - When a new worktree or experiment folder is approved, register its purpose, branch, and owner agent in [WORKTREES.md](WORKTREES.md) and [CODE_INDEX.md](CODE_INDEX.md) (keeping machine-specific paths confined to `WORKTREES.md` per Section 15).
   - Before finishing a task, check for and clean up unintended new paths, temporary files (after verifying ownership), and duplicate outputs.
3. **Protection of Other Agents' Work and Experiment Records**:
   - Do not modify, move, stash, reset, clean, or remove working folders, uncommitted changes, or untracked files owned by other agents.
   - Never overwrite or delete existing experiment records (`experiments/*/runs/`, `results/`, `rosbag2` archives, `result.json`, `events.jsonl`), including failed or partial runs. Always use a new `run_id` directory for new test runs.
4. **Targeted Context Updates**:
   - Upon completing a task, update only the common documents affected by your changes ([CURRENT_STATUS.md](CURRENT_STATUS.md), [CODE_INDEX.md](CODE_INDEX.md), [WORKTREES.md](WORKTREES.md), [VALIDATION.md](VALIDATION.md), [HANDOFF.md](HANDOFF.md), and [DECISIONS.md](DECISIONS.md) only when the user approves a finalized design decision), maintaining the verification distinctions required by Sections 5.2 and 14.
   - If a shared document has conflicting concurrent edits or lacks write permission, provide a minimal diff patch in your report rather than overwriting it.

---

## 19. 실차(LIMO) 파일 관리 규칙 (2026-11-24 정리 이후)

머신 고유 경로는 [WORKTREES.md](WORKTREES.md)에만 둔다(§15, §18). 아래는 모든 실차에 공통인 규칙이다.

1. **프로젝트 루트는 LIMO 홈의 `Capstone1_limo/` 하나다.** ROS 2 워크스페이스는 기존 `Capstone1_limo/robot_ws`로 고정한다. **새로운 영구 ROS 2 워크스페이스를 만들지 않는다**(임시 검증용 워크스페이스는 사용 후 사용자 승인 하에 정리). 홈 직하에 새 프로젝트 폴더를 만들지 않는다.
2. 프로젝트 관련 신규 실험 기록·로그·지도·rosbag·배포 후보·백업은 프로젝트 루트 하위(`deploy_candidates_*`, `deploy_backups_*`, `archive/<날짜>/`, `records/{mapping,validation,video,maps}/`, `tools/`)에서 관리한다. 기존 후보·롤백 디렉터리 이름은 바꾸지 않는다.
3. **인증 정보(`~/.config/limo_*`)와 안전 상태(Watchdog 속도 제한 상태 파일, Gate epoch, 장치 식별 설정)는 별도 변경 승인 전까지 기존 위치를 유지한다.** 이동·복사·출력하지 않는다.
4. **학교 및 팀원 환경을 보호한다.** `capstone_ws`, `limo_capstone`, `agilex_ws`, `limo_ws`, `jin_ws`, `YDLidar-SDK`, `.bashrc`, udev, 벤더 드라이버, 휴지통은 변경하지 않는다. 소유가 불확실한 항목은 이동하지 않는다.
5. **삭제 금지가 기본이다.** 이동·삭제 전에는 (a) 노트북에 독립 사본을 만들고 해시를 대조하고, (b) 실제 복원 가능성을 검증하고, (c) 참조하는 스크립트·문서를 확인하고, (d) 이동 전후 해시·권한 비교와 역방향 복구 목록을 남긴다. `mv -n`만 사용한다. 해시 목록만으로, 또는 같은 데이터를 가리키는 하드링크 복원본만으로 백업이라고 하지 않는다.
6. **대용량 실험 원본(rosbag, run 기록)은 노트북과 Google Drive에 별도 보관한다.** 보관 여부와 검증 상태를 [VALIDATION.md](VALIDATION.md)에 구분하여 기록하고, 검증하지 않은 보관을 완료로 적지 않는다(§17).
7. 과거 실험·시험 기록에 적힌 당시 경로는 역사적 기록이므로 고치지 않는다. 현행 절차 문서와 스크립트만 새 경로로 갱신하며, 스크립트 수정은 변경 범위와 검증 방법을 먼저 확인한 뒤 한다.
8. 실행 상태(Agent `state_dir`, `latest` 포인터, `run/*.pid`)는 해당 프로세스가 없고 참조 스크립트가 갱신된 뒤에만 이동한다. 새 실행 상태·기록의 저장 위치는 코드의 기본값에 맡기지 않고 매개변수·환경변수로 명시한다.
