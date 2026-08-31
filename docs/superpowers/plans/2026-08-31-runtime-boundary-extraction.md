# Runtime Boundary Extraction Implementation Plan

> **Execution:** Use subagent-driven development with TDD, specification review,
> and code-quality review after each task.

**Goal:** Convert the exact imported P450-SIM snapshot into a small, traceable
runtime-only foundation by removing benchmark/validation entrypoints from the
SIM copy, preserving immutable import provenance, and enforcing an offline
non-paper boundary gate.

**Scope:** All writes stay in `/media/lu/P450_PAPER/SIM/p450_sim_v1`.
`/media/lu/P450_PAPER/P450-PAPER` remains read-only. This plan does not build or
launch ROS, Gazebo, PX4, MAVROS, MoveIt, or the pick demo. It does not add paper
benchmark, Pilot, T2, Formal, RBP, M5, Task-aware, evidence, or lifecycle
infrastructure.

**Baseline:** `6baad5413912340a6ed6d143d86466eee1c559e9` contains the exact
867-file source snapshot bound to upstream commit
`6809c15e3919d1aa3acb6518ad61c49e4150435f`.

**Traceability model:** `config/import_provenance.json` remains immutable and
continues to describe the imported upstream snapshot. A new
`config/runtime_overlay.json` declares every removed or modified inherited
file. Tests prove that every current inherited file is either unchanged from
provenance or explicitly covered by the overlay; undeclared additions,
deletions, and modifications fail.

---

## Task 1: Extract the runtime slice with an overlay ledger

**Create:**

- `config/runtime_overlay.json`
- `tests/test_runtime_extraction.py`

**Modify:**

- `tests/test_imported_layout.py`
- `src/p450/brick_aerial_perception/CMakeLists.txt`
- `src/p450/brick_aerial_perception/launch/m1_aerial_perception.launch`
- `src/p450/brick_aerial_perception/README.md`
- `src/p450/brick_aerial_perception/test/test_ros_contract.py`
- `src/ground/bunker_navigation/CMakeLists.txt`
- `src/ground/bunker_navigation/test/test_configuration.py`
- `src/ground/brick_rgbd_perception/CMakeLists.txt`
- `src/ground/brick_visual_pick/CMakeLists.txt`
- `src/ground/brick_visual_pick/test/test_configuration.py`
- `src/ground/ground_pick_orchestrator/CMakeLists.txt`
- `src/ground/ground_pick_orchestrator/test/test_configuration.py`
- `src/ground/ground_pick_orchestrator/config/mission.yaml`
- `src/ground/ground_pick_orchestrator/scripts/ground_pick_node.py`

### Step 1: Write the failing extraction contract

`tests/test_runtime_extraction.py` must freeze the exact 42-file removal set,
the exact 13 inherited modified files, and this overlay schema:

```json
{
  "schema_version": 1,
  "base_provenance": "config/import_provenance.json",
  "base_provenance_sha256": "6cc35f32f30cc1970350eb4daa6f678b35cedc900ba1c645c0c159bec97abd20",
  "upstream_commit": "6809c15e3919d1aa3acb6518ad61c49e4150435f",
  "removed": ["sorted inherited paths"],
  "modified": {"sorted inherited path": "current sha256"}
}
```

The tests must prove:

- `config/import_provenance.json` itself still has the frozen SHA-256
  `6cc35f32f30cc1970350eb4daa6f678b35cedc900ba1c645c0c159bec97abd20`
  before any overlay rule is evaluated;
- every removed path existed in base provenance and is absent now;
- every modified path existed in base provenance, exists now, differs from its
  upstream hash, and matches the overlay current hash;
- every other provenance path still exists and still matches its upstream hash;
- the current inherited regular-file set—defined as the union of all regular
  files below the 18 imported package destinations plus the 11 auxiliary
  destinations—is exactly `provenance - removed`, with no undeclared additions
  or deletions; platform/local files outside those roots are not overlay inputs;
- no CMake, launch, or active Python file references a removed basename/path;
- the five affected packages retain only existing installed scripts/tests;
- `formal_pose_*` is absent from the remaining ground runtime and the unchanged
  `/brick_pose` compatibility output is named `legacy_pick_pose_*`;
- `/ground_pick/approved_brick_pose` remains unchanged.

Run before editing the inherited tree:

```bash
env -i PATH=/usr/bin:/bin \
  /usr/bin/python3 -B -m unittest -v tests.test_runtime_extraction
```

Expected: RED because the overlay does not exist and benchmark files remain.

### Step 2: Remove the exact benchmark/validation closure

Use `apply_patch` deletions only. All paths below are repository-relative.

#### Aerial perception: remove 7

```text
src/p450/brick_aerial_perception/scripts/m1_validation_monitor.py
src/p450/brick_aerial_perception/config/validation_scenarios.yaml
src/p450/brick_aerial_perception/test/test_validation.py
src/p450/brick_aerial_perception/src/brick_aerial_perception/validation.py
src/p450/brick_aerial_perception/docs/TEST_REPORT.md
src/p450/brick_aerial_perception/scripts/run_m1_demo.bash
src/p450/brick_aerial_perception/scripts/aerial_viewpoint_mission_multiview.py
```

Remove their CMake install entries, validation args/node/config from
`m1_aerial_perception.launch`, the now-empty `docs` install entry, and old
workspace/report instructions from the package README. Keep aerial perception,
viewpoint mission, TF bridge, config, RViz, and world assets.
Remove the obsolete demo-runner-only test method from
`test/test_ros_contract.py`; retain all live ROS/perception contract tests.
The multiview wrapper is explicitly marked M5-only; remove its CMake install
entry while retaining the normal `aerial_viewpoint_mission.py` entrypoint.

#### BUNKER navigation: remove 15

```text
src/ground/bunker_navigation/scripts/approach_trial_monitor.py
src/ground/bunker_navigation/scripts/run_approach_matrix.py
src/ground/bunker_navigation/scripts/run_navigation_matrix.py
src/ground/bunker_navigation/scripts/navigation_trial_monitor.py
src/ground/bunker_navigation/config/navigation_scenarios.yaml
src/ground/bunker_navigation/config/approach_scenarios.yaml
src/ground/bunker_navigation/launch/navigation_trial.launch
src/ground/bunker_navigation/launch/approach_pose_trial.launch
src/ground/bunker_navigation/test/test_metrics.py
src/ground/bunker_navigation/test/test_approach_metrics.py
src/ground/bunker_navigation/src/bunker_navigation/metrics.py
src/ground/bunker_navigation/src/bunker_navigation/approach_metrics.py
src/ground/bunker_navigation/scripts/probe_rear_obstacle_safety.py
src/ground/bunker_navigation/launch/rear_obstacle_safety_probe.launch
src/ground/bunker_navigation/models/rear_gate_obstacle.sdf
```

Remove corresponding install/test entries and benchmark-only assertions from
`test/test_configuration.py`. Keep `approach_pose_node.py`, heading gate,
navigation demo, costmaps, planar-drive plugin, approach obstacle/world, and
their runtime/unit tests.
Also remove that file's `WORKSPACE` constant and Dockerfile assertion: the
container/workspace infrastructure is not imported into V1, and the retained
configuration test must not require it.

#### RGB-D perception: remove 5

```text
src/ground/brick_rgbd_perception/launch/pose_robustness_validation.launch
src/ground/brick_rgbd_perception/test/test_validation.py
src/ground/brick_rgbd_perception/src/brick_rgbd_perception/validation.py
src/ground/brick_rgbd_perception/scripts/validate_pose_robustness.py
src/ground/brick_rgbd_perception/config/brick_pose_robustness.yaml
```

Remove the validator install/test entries. Keep live RGB-D, pose estimation,
geometry, launch, RViz, and runtime rostests.

#### Visual pick: remove 9

```text
src/ground/brick_visual_pick/scripts/visual_pick_trial_monitor.py
src/ground/brick_visual_pick/config/visual_pick_scenarios.yaml
src/ground/brick_visual_pick/launch/visual_pick_trial.launch
src/ground/brick_visual_pick/test/test_matrix_report.py
src/ground/brick_visual_pick/test/test_trial_results.py
src/ground/brick_visual_pick/test/test_pose_metrics.py
src/ground/brick_visual_pick/src/brick_visual_pick/matrix_report.py
src/ground/brick_visual_pick/src/brick_visual_pick/pose_metrics.py
src/ground/brick_visual_pick/src/brick_visual_pick/trial_results.py
```

Remove corresponding install/test entries and the two matrix-only assertions in
`test/test_configuration.py`. Keep live pose gate, quality gate, and demo launch.

#### Ground pick orchestrator: remove 6

```text
src/ground/ground_pick_orchestrator/scripts/run_ground_pick_matrix.py
src/ground/ground_pick_orchestrator/scripts/ground_pick_trial_monitor.py
src/ground/ground_pick_orchestrator/config/mission_scenarios.yaml
src/ground/ground_pick_orchestrator/launch/ground_pick_trial.launch
src/ground/ground_pick_orchestrator/test/test_mission_metrics.py
src/ground/ground_pick_orchestrator/src/ground_pick_orchestrator/mission_metrics.py
```

Remove corresponding install/test/configuration assertions. Keep the core
orchestrator, mission state, safety guards, observation planning, demo launch,
and runtime tests.

### Step 3: Rename the misleading Formal compatibility symbol

Perform a behavior-preserving mechanical rename in the core ground runtime:

```text
formal_pose_pub       -> legacy_pick_pose_pub
~formal_pose_topic    -> ~legacy_pick_pose_topic
formal_pose_topic     -> legacy_pick_pose_topic
```

Keep the default topic `/brick_pose` and the approved output
`/ground_pick/approved_brick_pose`. Update retained configuration tests. Do not
remove the compatibility publisher in this task.

### Step 4: Create and validate the overlay

Use immutable base provenance for removed-file membership and upstream hashes.
Compute current hashes for the 13 modified inherited files, then add the sorted
overlay with `apply_patch`. Modify `tests/test_imported_layout.py` so its
integrity test applies overlay semantics instead of requiring every current file
to equal the import snapshot.

Run:

```bash
env -i PATH=/usr/bin:/bin \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_imported_layout tests.test_runtime_extraction

env -i PATH=/usr/bin:/bin \
  /usr/bin/python3 -B -m unittest discover -s tests -v
```

Also run retained configuration tests directly with system Python 3.8 and
verify the upstream repository remains clean.

```bash
env -i PATH=/usr/bin:/bin /usr/bin/python3 -B \
  src/p450/brick_aerial_perception/test/test_ros_contract.py
env -i PATH=/usr/bin:/bin /usr/bin/python3 -B \
  src/ground/bunker_navigation/test/test_configuration.py
env -i PATH=/usr/bin:/bin /usr/bin/python3 -B \
  src/ground/brick_visual_pick/test/test_configuration.py
env -i PATH=/usr/bin:/bin /usr/bin/python3 -B \
  src/ground/ground_pick_orchestrator/test/test_configuration.py
```

### Step 5: Commit

Commit message:

```text
refactor: extract runtime from benchmark artifacts
```

After the commit, request specification review followed by code-quality review.

---

## Task 2: Enforce the offline non-paper boundary

**Create:**

- `tools/runtime_boundary.py`
- `tests/test_runtime_boundary.py`
- `.gitignore`
- `README.md`

### Step 1: Write failing scanner tests

The test suite must cover:

- recursive discovery of every `src/**/package.xml`, rejecting unlisted,
  duplicate, nested, or name/path-mismatched packages;
- current foundation package set equals the 18 imported packages;
- no dependency element references a forbidden package;
- no active runtime file contains configured method tokens;
- no active runtime code calls Gazebo delete, dynamic spawn, or respawn APIs;
- comments are excluded without changing finding line numbers for XML, Jinja,
  C/C++, Python, YAML, and CMake comments;
- active files exclude `docs`, `test`, `tests`, `results`, `generated`, caches,
  licenses, provenance, and overlay metadata;
- every inherited `gazebo_ros` `spawn_model` launch node is classified exactly
  once as joint candidate, standalone smoke, or inactive legacy; new
  unclassified startup spawns fail;
- no removed benchmark path or entrypoint is referenced by an active file.

Run and confirm RED because `tools.runtime_boundary` does not exist.

### Step 2: Implement the minimal scanner

Use only Python 3.8 standard library. Return deterministic frozen findings with
repository-relative path, source line, and token. Parse package dependencies and
launch XML structurally. Preserve newline counts while blanking block comments;
respect quoted strings when removing line comments.

The active textual allowlist is exact:

```text
CMakeLists.txt, *.cmake
*.launch, *.xml, *.xacro, *.urdf, *.sdf, *.world, *.jinja, *.config
*.py, *.cpp, *.cc, *.c, *.hpp, *.hh, *.h
*.yaml, *.yml, *.json, *.rviz
*.msg, *.srv, *.action
*.bash, *.sh
```

Anything not in that allowlist is not decoded. In particular, mesh, image,
shared-library, archive, point-cloud, numeric CSV, and material assets are
ignored as non-executable data. Tests must place forbidden-looking bytes in a
`.png` fixture and prove it is ignored, while parameterizing every textual
suffix above to prove it is discovered.

Comment removal is selected by language family:

- XML-like launch/xacro/URDF/SDF/world/config files: `<!-- -->`;
- Jinja: XML comments and `{# #}`;
- C/C++: `/* */` and quote-aware `//`;
- Python: standard-library `tokenize` COMMENT tokens;
- YAML/RViz/CMake/msg/srv/action/shell: quote-aware `#`;
- JSON: no comments.

All block-comment blanking must preserve every newline so finding line numbers
remain stable.

Dynamic lifecycle tokens include:

```text
/gazebo/delete_model
DeleteModel
SpawnModel
/gazebo/spawn_urdf_model
/gazebo/spawn_sdf_model
respawn_model
delete_respawn
```

Configured method tokens are matched case-insensitively. In addition, apply the
identifier-boundary pattern
`(?<![A-Za-z0-9])m5(?:[_/-]|(?![A-Za-z0-9]))` with `IGNORECASE`; it catches
`M5-only`, `m5_`, and `/m5/` without matching `param5`. Add all four as explicit
regression cases.

`gazebo_ros` launch nodes of type `spawn_model` are startup operations, not
dynamic lifecycle calls. Freeze these exact twelve post-extraction `(path,
node-name)` records. The three sets must be exhaustive and pairwise disjoint;
any new unclassified node fails.

Joint candidates:

```text
src/p450/prometheus_gazebo/launch_basic/sitl_px4_outdoor.launch | $(arg vehicle)_$(arg uav_id)_spawn
src/ground/bunker_aubo_gazebo/launch/combined_robot.launch | spawn_bunker_aubo
src/ground/bunker_aubo_gazebo/launch/combined_robot.launch | spawn_bunker_aubo_startup_pose
```

Standalone smoke only:

```text
src/ground/bunker_aubo_gazebo/launch/ag95_only.launch | spawn_ag95
src/ground/bunker_aubo_gazebo/launch/aubo_only.launch | spawn_aubo_i5
src/ground/bunker_aubo_gazebo/launch/bunker_only.launch | spawn_bunker
```

Inactive legacy wrappers:

```text
src/ground/bunker_aubo_gazebo/launch/brick_world.launch | spawn_brick
src/ground/ground_pick_orchestrator/launch/ground_pick_demo.launch | spawn_ground_pick_brick
src/ground/ground_pick_orchestrator/launch/ground_pick_demo.launch | spawn_ground_pick_obstacle
src/p450/prometheus_gazebo/launch_basic/sitl_px4_indoor.launch | $(arg vehicle)_$(arg uav_id)_spawn
src/vendor/aubo_description/launch/gazebo.launch | spawn_gazebo_model
src/vendor/dh_ag95_description/launch/gazebo.launch | spawn_gazebo_model
```

These inherited nodes are not V1 platform entrypoints. The later bringup task
must replace this classification with its own joint/standalone entrypoint
contract.

### Step 3: Add repository boundary documentation

`.gitignore` contains only generated/local outputs:

```gitignore
build/
devel/
install/
logs/
.catkin_tools/
__pycache__/
*.pyc
.import-staging-*/
```

`README.md` must state:

- P450-PAPER is immutable upstream;
- source enters through the frozen manifest and dry-run-first importer;
- import provenance plus runtime overlay provide traceability;
- benchmark/validation artifacts and all paused paper methods are outside V1;
- inherited launch wrappers are not platform entrypoints;
- the target stack is Ubuntu 20.04, ROS Noetic, Gazebo 11, PX4, MAVROS, and
  MoveIt;
- the offline gate proves only source selection and boundary integrity, not
  build, launch, TF, sensor, flight, manipulation, or demo success.

### Step 4: Run the foundation gate

```bash
env -i PATH=/usr/bin:/bin \
  /usr/bin/python3 -B -m unittest discover -s tests -v

git status --short
git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM status --short
```

Expected: all tests pass, SIM status contains only Task 2 changes before commit,
and upstream status is empty. After adding `.gitignore`, existing cache files no
longer appear in status; do not delete them merely to make the gate green.

### Step 5: Commit

Commit message:

```text
test: enforce non-paper runtime boundary
```

Request specification review, code-quality review, and then run the complete
foundation completion gate before planning the P450 clean build.

---

## Foundation completion gate

Required evidence:

- feature branch is clean;
- all Python 3.8 offline tests pass from an empty environment;
- upstream repository remains clean at `6809c15e...`;
- every current inherited file is provenance-identical or overlay-declared;
- all 42 benchmark/validation/M5-only files are absent;
- exactly 18 inherited packages exist and no forbidden dependency/token is
  active;
- dynamic delete/spawn/respawn APIs are absent;
- all inherited startup spawns are explicitly classified;
- no claim is made yet about ROS build or runtime behavior.

Only after this gate may the next plan start the clean P450 build. Ground and
shared-world implementation remain separate reviewed milestones.
