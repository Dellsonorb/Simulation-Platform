# Deterministic RM4D to SIM Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Connect one sensor-derived P450 Brick observation to the frozen external RM4D `BasePlacementAPI`, visualize its unchanged top-K BUNKER poses, navigate top-1, refine with the Ground D435 from an exact target-facing pregrasp, and complete the existing physical grasp and Lift.

**Architecture:** A new `rm4d_sim_integration` ROS package runs directly under the external frozen RM4D Python environment. A small core converts exact map-frame TCP poses and current BUNKER SE(2) state into the frozen Python request, applying only the fixed local-Y `+1e-6 rad` query regularization. The existing Air-Ground orchestrator gains an explicit `rm4d` placement mode while its legacy standoff mode remains available, and all navigation/manipulation execution stays in the current common runtime.

**Tech Stack:** ROS Noetic/catkin, Python 3, `rospy`, `tf2_ros`, custom ROS service generation, `geometry_msgs`, `visualization_msgs`, frozen external RM4D Python 3.10 API, MoveBaseAction, MoveIt, Gazebo Classic, pytest/nosetests.

---

## File structure

Create `src/integrations/rm4d_sim_integration/` with focused responsibilities:

- `CMakeLists.txt`, `package.xml`, `setup.py`: catkin package and service generation.
- `srv/PlanBasePlacement.srv`: exact ROS request/response contract.
- `src/rm4d_sim_integration/geometry.py`: finite-value validation, quaternion multiplication, fixed regularization, yaw/pose conversion, and ordered candidate conversion.
- `src/rm4d_sim_integration/core.py`: external revision validation and injected `BasePlacementAPI` call; no ROS globals.
- `src/rm4d_sim_integration/markers.py`: pure marker geometry/specification for ranked BUNKER footprints.
- `scripts/rm4d_adapter_node.py`: ROS service, TF lookup, publishers, and one persistent external API instance.
- `scripts/replay_rm4d_observation.py`: one offline/online replay CLI using the same core.
- `config/p450_rgbd_replay.yaml`: one actual sensor-derived P450 observer result and the grasp parameters needed to reproduce its exact TCP.
- `launch/rm4d_adapter.launch`: explicit external checkout/interpreter/map configuration.
- `launch/rm4d_air_ground_pick_demo.launch`: compose the existing SIM demo components with RM4D placement mode.
- `rviz/rm4d_candidates.rviz`: minimal `map`-fixed candidate/TCP displays.
- `test/test_geometry.py`, `test/test_core.py`, `test/test_contract.py`: focused unit and package-contract tests.

Modify the existing demo without changing its default legacy mode:

- `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/approach.py`: exact candidate staging helper.
- `src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py`: optional RM4D service call, top-1 execution, and aerial-derived target-facing pregrasp observation.
- `src/demos/air_ground_pick_demo/config/demo.yaml`: explicit default placement mode and RM4D parameters.
- `src/demos/air_ground_pick_demo/package.xml`: runtime dependency on `rm4d_sim_integration`.
- `src/demos/air_ground_pick_demo/test/test_approach.py` and `test/test_orchestrator_contract.py`: new mode behavior.
- `tests/test_air_ground_pick_demo.py`: repository-level integration contract.
- `scripts/smoke_rm4d_air_ground_pick_demo.bash`: bounded natural E2E runner.
- `README.md`: external dependency setup, replay, RViz, and natural-demo commands.

### Task 1: Pure fixed-regularization and candidate conversion core

**Files:**
- Create: `src/integrations/rm4d_sim_integration/CMakeLists.txt`
- Create: `src/integrations/rm4d_sim_integration/package.xml`
- Create: `src/integrations/rm4d_sim_integration/setup.py`
- Create: `src/integrations/rm4d_sim_integration/src/rm4d_sim_integration/__init__.py`
- Create: `src/integrations/rm4d_sim_integration/src/rm4d_sim_integration/geometry.py`
- Create: `src/integrations/rm4d_sim_integration/test/test_geometry.py`
- Create: `tests/test_rm4d_sim_integration.py`

- [ ] **Step 1: Write failing tests for the frozen numerical rule**

```python
def test_regularization_is_fixed_local_y_and_does_not_mutate_exact_pose():
    exact = PoseValues((2.0, 0.0, 0.0794), top_down_quaternion(0.0))
    query = regularize_for_rm4d(exact)
    assert exact.position == (2.0, 0.0, 0.0794)
    assert quaternion_angle(exact.orientation, query.orientation) == pytest.approx(1e-6)
    assert query.position == exact.position

def test_request_rejects_non_map_frame():
    with pytest.raises(IntegrationError, match="must be map"):
        build_rm4d_request("odom", exact_pose(), (3.0, 0.0, 0.0), "brick-001")
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
python3 -m pytest -q src/integrations/rm4d_sim_integration/test/test_geometry.py
```

Expected: collection/import failure because `rm4d_sim_integration.geometry`
does not exist.

- [ ] **Step 3: Implement only immutable pose values, fixed quaternion multiplication, request creation, and ordered candidate conversion**

The public numerical function has no epsilon parameter:

```python
RM4D_LOCAL_Y_REGULARIZATION_RAD = 1e-6

def regularize_for_rm4d(exact_pose):
    local_y = (0.0, math.sin(0.5e-6), 0.0, math.cos(0.5e-6))
    return PoseValues(exact_pose.position,
                      normalized_multiply(exact_pose.orientation, local_y))
```

`build_rm4d_request(...)` always writes `frame_id="map"`, preserves XYZ,
uses the regularized quaternion only in the returned dictionary, and includes
only `grasp_id` and current BUNKER `x/y/yaw`. `ordered_candidates(...)` returns
the source order unchanged.

- [ ] **Step 4: Run focused and repository contract tests and verify GREEN**

```bash
PYTHONPATH=src/integrations/rm4d_sim_integration/src \
  python3 -m pytest -q \
  src/integrations/rm4d_sim_integration/test/test_geometry.py \
  tests/test_rm4d_sim_integration.py
```

Expected: all new tests pass and `git diff --check` is clean.

- [ ] **Step 5: Commit the pure integration boundary**

```bash
git add src/integrations/rm4d_sim_integration tests/test_rm4d_sim_integration.py
git commit -m "feat: add frozen RM4D integration geometry"
```

### Task 2: External API core and exact dependency boundary

**Files:**
- Create: `src/integrations/rm4d_sim_integration/src/rm4d_sim_integration/core.py`
- Create: `src/integrations/rm4d_sim_integration/test/test_core.py`

- [ ] **Step 1: Write failing tests using an injected fake API**

```python
def test_plan_passes_regularized_copy_and_preserves_rank():
    api = RecordingAPI(result_with_candidates("second", "first"))
    planner = IntegrationCore(api)
    result = planner.plan(exact_pose(), (3.0, 0.0, 0.0), "brick-001", 2)
    assert api.request["frame_id"] == "map"
    assert quaternion_angle(EXACT_Q, api.request["quaternion_xyzw"]) == pytest.approx(1e-6)
    assert [item.candidate_id for item in result.candidates] == ["second", "first"]

def test_exact_external_revision_is_required():
    with pytest.raises(IntegrationError, match="revision mismatch"):
        require_external_revision("fb2a845", FROZEN_COMMIT)
```

- [ ] **Step 2: Run the core test and verify RED**

```bash
PYTHONPATH=src/integrations/rm4d_sim_integration/src \
  python3 -m pytest -q src/integrations/rm4d_sim_integration/test/test_core.py
```

Expected: import failure for `rm4d_sim_integration.core`.

- [ ] **Step 3: Implement the injected core and external revision reader**

`IntegrationCore` calls only `api.plan(request, top_k=top_k)`. It accepts
`status=ok` and `status=no_feasible_candidate`, validates finite candidate
`bunker_x/y/yaw/final_score`, and retains list order. `read_external_revision`
uses `git -C <external-root> rev-parse HEAD`; it never runs fetch, checkout,
pull, clean, or any write operation.

- [ ] **Step 4: Verify GREEN**

```bash
PYTHONPATH=src/integrations/rm4d_sim_integration/src \
  python3 -m pytest -q src/integrations/rm4d_sim_integration/test
```

Expected: all integration-core tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/integrations/rm4d_sim_integration
git commit -m "feat: call frozen RM4D API through a thin core"
```

### Task 3: ROS service, top-K topics, and RViz markers

**Files:**
- Create: `src/integrations/rm4d_sim_integration/srv/PlanBasePlacement.srv`
- Create: `src/integrations/rm4d_sim_integration/src/rm4d_sim_integration/markers.py`
- Create: `src/integrations/rm4d_sim_integration/scripts/rm4d_adapter_node.py`
- Create: `src/integrations/rm4d_sim_integration/launch/rm4d_adapter.launch`
- Create: `src/integrations/rm4d_sim_integration/rviz/rm4d_candidates.rviz`
- Create: `src/integrations/rm4d_sim_integration/test/test_contract.py`
- Modify: `src/integrations/rm4d_sim_integration/CMakeLists.txt`
- Modify: `src/integrations/rm4d_sim_integration/package.xml`

- [ ] **Step 1: Write failing service/launch/marker contract tests**

The tests assert the exact service fields, one reusable API initialization,
`map` and `ground/base_link` defaults, latched publishers, top-1 color, padded
footprint dimensions, and absence of the strings `/gazebo`, `world`,
`ModelStates`, and direct controller/plugin commands in the runtime node.

- [ ] **Step 2: Run and verify RED**

```bash
PYTHONPATH=src/integrations/rm4d_sim_integration/src \
  python3 -m pytest -q src/integrations/rm4d_sim_integration/test/test_contract.py
```

Expected: missing service, node, launch, and marker files.

- [ ] **Step 3: Add the ROS service and minimal node**

Use this exact service definition:

```text
geometry_msgs/PoseStamped grasp_tcp
string grasp_id
uint32 top_k
---
bool success
string status
geometry_msgs/PoseArray candidates
string[] candidate_ids
float64[] scores
string message
```

The node validates the frozen checkout, prepends only that external root to
`sys.path`, imports `BasePlacementAPI`, loads it once, and handles service calls
with current TF-derived BUNKER SE(2). It publishes the exact input pose plus
ranked `PoseArray` and `MarkerArray` in `map`.

- [ ] **Step 4: Build messages and verify GREEN**

```bash
scripts/with_noetic_env.bash catkin build rm4d_sim_integration --no-status
source devel/setup.bash
python3 -m pytest -q src/integrations/rm4d_sim_integration/test
rossrv show rm4d_sim_integration/PlanBasePlacement
```

Expected: package builds, all tests pass, and the generated service matches the
frozen contract.

- [ ] **Step 5: Commit**

```bash
git add src/integrations/rm4d_sim_integration
git commit -m "feat: expose RM4D candidates through ROS"
```

### Task 4: Sensor-derived offline/online replay

**Files:**
- Create: `src/integrations/rm4d_sim_integration/config/p450_rgbd_replay.yaml`
- Create: `src/integrations/rm4d_sim_integration/scripts/replay_rm4d_observation.py`
- Modify: `src/integrations/rm4d_sim_integration/CMakeLists.txt`
- Modify: `src/integrations/rm4d_sim_integration/test/test_contract.py`

- [ ] **Step 1: Write failing replay tests**

```python
def test_replay_source_is_sensor_observation_in_map():
    replay = yaml.safe_load(REPLAY.read_text())
    assert replay["source_topic"] == "/air_observer/target_pose"
    assert replay["frame_id"] == "map"
    assert "gazebo" not in json.dumps(replay).lower()

def test_replay_uses_existing_side_up_grasp_definition():
    source = REPLAY_SCRIPT.read_text()
    assert "generate_top_down_grasp" in source
    assert "PlanBasePlacement" in source
```

- [ ] **Step 2: Run and verify RED**

Expected: replay fixture/script missing.

- [ ] **Step 3: Add one actual P450 observer record and dual-mode CLI**

Use the prior sensor-derived `AIR_HANDOFF` record as the initial replay source:

```yaml
source_topic: /air_observer/target_pose
frame_id: map
target: [1.9899721371390122, 0.004464723547961525,
         0.023386687889514802, 0.008026569019304208]
current_bunker_pose: [3.0, 0.0, 0.0]
```

The CLI imports the existing side-up grasp helper. Offline mode creates one
external `BasePlacementAPI` and calls `IntegrationCore`; online mode calls
`/rm4d/plan_base_placement`. Both print the same ordered candidate summary.

- [ ] **Step 4: Verify tests, then execute real offline and online replay**

Use the exact detached checkout and existing 10M map. Compare status,
candidate IDs, XY/yaw, and scores without tolerance-based reordering.

- [ ] **Step 5: Commit**

```bash
git add src/integrations/rm4d_sim_integration
git commit -m "feat: replay P450 observations through RM4D"
```

### Task 5: RM4D top-1 and target-facing pregrasp in the natural demo

**Files:**
- Modify: `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/approach.py`
- Modify: `src/demos/air_ground_pick_demo/test/test_approach.py`
- Modify: `src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py`
- Modify: `src/demos/air_ground_pick_demo/config/demo.yaml`
- Modify: `src/demos/air_ground_pick_demo/package.xml`
- Modify: `src/demos/air_ground_pick_demo/test/test_orchestrator_contract.py`
- Modify: `tests/test_air_ground_pick_demo.py`
- Create: `src/integrations/rm4d_sim_integration/launch/rm4d_air_ground_pick_demo.launch`

- [ ] **Step 1: Write failing exact-candidate navigation tests**

```python
def test_staged_candidate_finishes_at_exact_rm4d_pose():
    position, final = compute_staged_candidate_goals(
        (3.0, 0.0, 0.0), (2.575, 0.375, 0.0))
    assert position == GoalGeometry(2.575, 0.375, 0.0)
    assert final == GoalGeometry(2.575, 0.375, 0.0)
```

Add orchestrator contract assertions that `rm4d` mode calls the service after
`AIR_HANDOFF`, sends response pose zero unchanged to MoveBaseAction, and does
not call `compute_standoff_goal` or change candidate yaw.

- [ ] **Step 2: Run and verify RED**

```bash
PYTHONPATH=src/demos/air_ground_pick_demo/src \
  python3 -m pytest -q \
  src/demos/air_ground_pick_demo/test/test_approach.py \
  src/demos/air_ground_pick_demo/test/test_orchestrator_contract.py \
  tests/test_air_ground_pick_demo.py
```

Expected: missing candidate helper and RM4D mode.

- [ ] **Step 3: Implement top-1 mode and exact aerial-derived observation/pregrasp**

Keep `placement_mode: standoff` as the legacy default. The RM4D launch sets
`placement_mode: rm4d`, waits for the service, generates the exact aerial
grasp, calls RM4D, and navigates candidate zero. After ground stop it opens the
AG95, uses the existing exact generated pregrasp as the MoveIt observation
target with a current timestamp, then waits for a fresh Ground D435 pose. The
refined pose continues through the existing exact grasp and Lift code.

The RM4D path never invokes `_move_to_ground_observation()` or the old fixed
joint posture. It publishes `RM4D_CANDIDATES` and includes top-1 ID, score, and
map pose in `GROUND_APPROACH`/`GROUND_STOPPED` status.

- [ ] **Step 4: Verify GREEN and legacy regressions**

```bash
scripts/with_noetic_env.bash catkin build \
  rm4d_sim_integration air_ground_pick_demo --no-status
python3 -m pytest -q \
  src/demos/air_ground_pick_demo/test \
  src/integrations/rm4d_sim_integration/test \
  tests/test_air_ground_pick_demo.py \
  tests/test_sim_to_real_interface_contract.py
```

Expected: all focused tests pass; legacy standoff configuration remains valid.

- [ ] **Step 5: Commit**

```bash
git add src/demos/air_ground_pick_demo \
  src/integrations/rm4d_sim_integration/launch \
  tests/test_air_ground_pick_demo.py
git commit -m "feat: drive Air-Ground Pick from RM4D top one"
```

### Task 6: Natural replay, RViz, navigation, and E2E verification

**Files:**
- Create: `scripts/smoke_rm4d_air_ground_pick_demo.bash`
- Modify: `README.md`
- Modify only as required by a new failing regression test: integration/demo
  files from Tasks 1-5.

- [ ] **Step 1: Write the failing smoke/README contract test**

Extend `tests/test_rm4d_sim_integration.py` to require a bounded smoke script,
the exact external dependency arguments, the RViz command, and terminal
`RM4D_SIM_INTEGRATION_READY` semantics.

- [ ] **Step 2: Run and verify RED**

Expected: smoke script and README instructions missing.

- [ ] **Step 3: Add the bounded runner and concise documentation**

The smoke script launches `rm4d_air_ground_pick_demo.launch`, waits for
`RM4D_CANDIDATES`, `GROUND_STOPPED`, and `LIFT`, and succeeds only when all are
observed in order. It terminates the launch cleanly on completion or timeout.
It does not inspect Gazebo model states or fabricate sensor input.

- [ ] **Step 4: Run full verification in increasing cost order**

```bash
python3 -m pytest -q tests src/integrations/rm4d_sim_integration/test \
  src/demos/air_ground_pick_demo/test
scripts/with_noetic_env.bash catkin build --no-status
```

Then run:

1. offline replay against the exact external checkout and 10M map;
2. online ROS replay and inspect `/rm4d/candidates` and
   `/rm4d/candidate_markers`;
3. RViz with fixed frame `map` and the committed display config;
4. a clear-scene run through top-1 `GROUND_STOPPED`;
5. one natural Gazebo run through `LIFT`.

If an execution bug appears, write a focused failing test before changing
production code, make the smallest fix, and repeat the affected stage.

- [ ] **Step 5: Commit verified integration and stop**

```bash
git add README.md scripts/smoke_rm4d_air_ground_pick_demo.bash \
  tests/test_rm4d_sim_integration.py
git commit -m "docs: verify RM4D driven Air-Ground Pick"
```

After all tests and the natural E2E pass, emit exactly:

```text
RM4D_SIM_INTEGRATION_READY
```

Do not proceed into RM4D optimization, uncertainty, MID360, NBV, Task-aware
methods, benchmark infrastructure, evidence systems, or additional safety
framework work.
