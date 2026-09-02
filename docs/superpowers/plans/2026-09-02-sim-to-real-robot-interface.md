# Sim-to-Real Robot Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the existing Air-Ground Pick Demo consume a small SIM/REAL-common robot contract while preserving Prometheus, `move_base`, official BUNKER ROS topics, MoveIt, and the existing manipulation controllers as the real control authorities.

**Architecture:** Add only one new message package and one thin P450 facade package. Keep native Prometheus state topics public, place `move_base` above the BUNKER `/cmd_vel + odom + BunkerStatus` boundary, isolate Gazebo-only localization/contact/vehicle behavior in SIM packages, and expose `map` rather than `world` to common runtime and task code.

**Tech Stack:** ROS 1 Noetic, catkin, Python 3/rospy/actionlib/tf2, C++ Gazebo ModelPlugin, Prometheus/PX4 SITL/MAVROS, move_base, MoveIt, unittest/rostest.

---

## Governing specification

Implement [the approved design specification](../specs/2026-09-02-sim-to-real-robot-interface-design.md). Do not add benchmark, evidence, paper, formal, Pilot, lifecycle, or adversarial-safety infrastructure.

The following boundaries are non-negotiable:

- `FlightCommand` translates to Prometheus messages and observes native state; it does not implement flight control or a second flight state machine.
- `MoveBaseAction` is a SIM/REAL-common navigation interface, not part of the BUNKER hardware adapter.
- The BUNKER hardware boundary is `/ground/cmd_vel`, `/ground/odom`, `/ground/bunker_status`, and `ground/odom -> ground/base_link`.
- `map -> uav1/odom` and `map -> ground/odom` are Localization Adapter Contract edges with exactly one authority each.
- `world` and Gazebo contact topics remain inside SIM launch/adapters and test oracles.
- The public grasp semantic is `/ground/gripper/grasp_confirmed`, never bilateral contact.
- SIM BUNKER implements ROS interface fidelity only; it does not emulate `ugv_sdk`, CAN, or motor electronics.

## Task 1: Define the public ROS contracts

**Files:**

- Create: `src/platform/robot_runtime_interfaces/CMakeLists.txt`
- Create: `src/platform/robot_runtime_interfaces/package.xml`
- Create: `src/platform/robot_runtime_interfaces/action/FlightCommand.action`
- Create: `src/vendor/bunker_msgs/CMakeLists.txt`
- Create: `src/vendor/bunker_msgs/package.xml`
- Create: `src/vendor/bunker_msgs/msg/BunkerMotorState.msg`
- Create: `src/vendor/bunker_msgs/msg/BunkerStatus.msg`
- Test: `tests/test_sim_to_real_interface_contract.py`

**Step 1: Write the failing contract test**

Add assertions that:

- `FlightCommand.action` defines only `TAKEOFF`, `FLY_TO`, `HOVER`, `LAND`, `command`, and `geometry_msgs/PoseStamped target` in its goal; `success/message` in its result; and `current_pose/position_error/native_mode` in feedback.
- The action package depends on `actionlib_msgs`, `geometry_msgs`, and `message_generation/message_runtime`.
- The vendored `bunker_msgs` definitions match the official ROS 1 field names and types used by `BunkerStatus`, including the official `BunkerMotorState[] motor_states` field.
- No locally invented universal robot state or Gazebo type appears in either contract package.

**Step 2: Run the test to verify RED**

Run:

```bash
/usr/bin/python3 -m unittest -q tests.test_sim_to_real_interface_contract
```

Expected: FAIL because both packages are absent.

**Step 3: Add the minimal message packages**

Create the action exactly as frozen in the design. Vendor the official message-only `bunker_msgs` definitions needed by the SIM adapter; do not vendor or emulate `bunker_base`, `ugv_sdk`, or CAN.

**Step 4: Build and verify GREEN**

Run:

```bash
/usr/bin/python3 -m unittest -q tests.test_sim_to_real_interface_contract
./scripts/with_noetic_env.bash catkin_make
```

Expected: contract test passes and catkin generates both action and status messages.

**Step 5: Commit**

```bash
git add src/platform/robot_runtime_interfaces src/vendor/bunker_msgs tests/test_sim_to_real_interface_contract.py
git commit -m "feat: define common robot runtime contracts"
```

## Task 2: Implement the thin Prometheus flight facade

**Files:**

- Create: `src/platform/p450_flight_facade/CMakeLists.txt`
- Create: `src/platform/p450_flight_facade/package.xml`
- Create: `src/platform/p450_flight_facade/setup.py`
- Create: `src/platform/p450_flight_facade/src/p450_flight_facade/__init__.py`
- Create: `src/platform/p450_flight_facade/src/p450_flight_facade/translation.py`
- Create: `src/platform/p450_flight_facade/scripts/p450_flight_facade.py`
- Create: `src/platform/p450_flight_facade/launch/p450_flight_facade.launch`
- Create: `src/platform/p450_flight_facade/test/test_translation.py`
- Modify: `tests/test_sim_to_real_interface_contract.py`

**Step 1: Write failing pure translation tests**

Test that pure helpers:

- validate only four public commands;
- map takeoff to native setup/control/mode plus `Init_Pos_Hover` without MAVROS setpoints;
- map fly-to to Prometheus `Move + XYZ_POS` after a caller-provided frame transform;
- map hover to `Current_Pos_Hover` and land to `Land`;
- treat cancel of fly-to as hover;
- contain no timer-driven flight-state transitions or SIM-specific operation deadline.

Add structural assertions that the facade imports `prometheus_msgs`, not `mavros_msgs` or `gazebo_msgs`, and publishes only Prometheus command/setup topics.

**Step 2: Run the tests to verify RED**

Run:

```bash
/usr/bin/python3 src/platform/p450_flight_facade/test/test_translation.py -q
/usr/bin/python3 -m unittest -q tests.test_sim_to_real_interface_contract
```

Expected: FAIL because the package and translator do not exist.

**Step 3: Implement the smallest action facade**

Implement `/uav1/runtime/flight` with `actionlib.SimpleActionServer`:

- read native `UAVState`, `UAVControlState`, and `/uav1/prometheus/odom`;
- publish native `UAVSetup` and `UAVCommand`;
- use TF only to transform `FLY_TO` targets into `uav1/odom`;
- use native state/odom solely to report completion and feedback;
- abort on stale/missing backend, invalid odometry, or native failsafe;
- translate preemption of active flight motion to native hover;
- never publish MAVROS/PX4 setpoints and never implement a controller.

Keep caller/task deadlines outside the facade. Expose tolerances and state-freshness as ordinary runtime parameters, not lifecycle/safety machinery.

**Step 4: Verify GREEN and build**

Run:

```bash
/usr/bin/python3 src/platform/p450_flight_facade/test/test_translation.py -q
/usr/bin/python3 -m unittest -q tests.test_sim_to_real_interface_contract
./scripts/with_p450_env.bash catkin_make
```

Expected: tests pass and the action server package builds in the P450 overlay.

**Step 5: Commit**

```bash
git add src/platform/p450_flight_facade tests/test_sim_to_real_interface_contract.py
git commit -m "feat: add thin Prometheus flight facade"
```

## Task 3: Establish the Localization Adapter Contract

**Files:**

- Modify: `src/platform/sim_platform_bringup/launch/p450_runtime.launch`
- Modify: `src/platform/sim_platform_bringup/launch/p450_standalone.launch`
- Modify: `src/platform/sim_platform_bringup/launch/air_ground_standalone.launch`
- Modify: `src/platform/ground_manipulator_runtime/launch/ground_robot_runtime.launch`
- Modify: `src/platform/ground_manipulator_runtime/launch/ground_robot_standalone.launch`
- Modify: `src/platform/bunker_sim_runtime/launch/bunker_runtime.launch`
- Modify: `src/platform/bunker_sim_runtime/launch/bunker_standalone.launch`
- Modify: `src/platform/sim_platform_bringup/config/p450_tf_contract.yaml`
- Modify: `src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml`
- Test: `tests/test_air_ground_platform.py`
- Test: `src/platform/sim_platform_bringup/test/test_p450_launch_contract.py`
- Test: `src/platform/bunker_sim_runtime/test/test_launch_contract.py`

**Step 1: Rewrite TF contract tests first**

Require runtime launches to be worldless and standalone SIM launches to own only these SIM localization edges:

```text
world -> map
map -> uav1/odom
map -> ground/odom
```

Require Prometheus to own `uav1/odom -> uav1/base_link` and the BUNKER adapter to own `ground/odom -> ground/base_link`. Add source-level uniqueness checks so each localization edge has one publisher in each launch composition.

**Step 2: Run focused tests to verify RED**

Run:

```bash
/usr/bin/python3 -m unittest -q tests.test_air_ground_platform
/usr/bin/python3 src/platform/sim_platform_bringup/test/test_p450_launch_contract.py -q
/usr/bin/python3 src/platform/bunker_sim_runtime/test/test_launch_contract.py -q
```

Expected: FAIL on existing `world -> */odom` runtime authorities and old P450 parent frame.

**Step 3: Move SIM anchors into standalone launch wrappers**

- Set Prometheus `tf_parent_frame` to `uav1/odom`.
- Remove `world` static transforms from robot runtime launches.
- Publish `world -> map` and spawn/calibration `map -> */odom` only in SIM standalone composition.
- Preserve configurable spawn transforms so the local odometry origins match the robot spawn poses.

**Step 4: Verify GREEN**

Run the focused tests again and ensure they pass.

**Step 5: Commit**

```bash
git add src/platform/sim_platform_bringup src/platform/ground_manipulator_runtime/launch src/platform/bunker_sim_runtime/launch tests/test_air_ground_platform.py
git commit -m "refactor: isolate SIM localization adapters"
```

## Task 4: Match the official BUNKER ROS hardware boundary

**Files:**

- Modify: `src/platform/bunker_sim_runtime/src/bunker_planar_move_plugin.cpp`
- Modify: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/velocity_guard.py`
- Modify: `src/platform/bunker_sim_runtime/scripts/velocity_guard.py`
- Modify: `src/platform/bunker_sim_runtime/launch/bunker_runtime.launch`
- Modify: `src/platform/bunker_sim_runtime/CMakeLists.txt`
- Modify: `src/platform/bunker_sim_runtime/package.xml`
- Modify: `src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro`
- Modify: `src/platform/ground_manipulator_runtime/package.xml`
- Test: `src/platform/bunker_sim_runtime/test/test_velocity_guard.py`
- Test: `src/platform/bunker_sim_runtime/test/test_renderer.py`
- Test: `tests/test_bunker_build_contract.py`
- Test: `tests/test_bunker_simple_smoke.py`

**Step 1: Write failing boundary tests**

Require:

- velocity guard input `/ground/nav_cmd_vel`, output `/ground/cmd_vel`;
- Gazebo plugin input `/ground/cmd_vel`;
- `/ground/odom` starts at local zero and is integrated from applied body motion rather than assigned from `WorldPose()`;
- plugin publishes `/ground/bunker_status` as `bunker_msgs/BunkerStatus`;
- only the plugin broadcasts `ground/odom -> ground/base_link`;
- there are no `ugv_sdk`, CAN, simulated CAN frame, or motor-electronics dependencies.

**Step 2: Verify RED**

Run:

```bash
/usr/bin/python3 src/platform/bunker_sim_runtime/test/test_velocity_guard.py -q
/usr/bin/python3 src/platform/bunker_sim_runtime/test/test_renderer.py -q
/usr/bin/python3 -m unittest -q tests.test_bunker_build_contract tests.test_bunker_simple_smoke
```

Expected: FAIL on old `cmd_vel_safe`, world-pose odometry, and missing status.

**Step 3: Implement interface-fidelity adapter**

- Change the velocity path to `nav_cmd_vel -> velocity guard -> cmd_vel`.
- Integrate local planar odometry using actual simulated base linear/angular velocity and simulation time.
- Publish odom, odom TF, and a minimal honest simulated `BunkerStatus` at the current update rate.
- Keep Gazebo velocity application inside the plugin; do not emulate official driver internals.
- Update both standalone and combined robot plugin XML generated by the existing renderer/xacro.

**Step 4: Build and verify GREEN**

Run focused tests and:

```bash
./scripts/with_noetic_env.bash catkin_make
```

Expected: adapter compiles and all BUNKER boundary tests pass.

**Step 5: Commit**

```bash
git add src/platform/bunker_sim_runtime src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro src/platform/ground_manipulator_runtime/package.xml tests/test_bunker_build_contract.py tests/test_bunker_simple_smoke.py
git commit -m "refactor: align SIM BUNKER ROS boundary"
```

## Task 5: Standardize P450 MID360 and add BUNKER IMU

**Files:**

- Modify: `src/p450/prometheus_gazebo/gazebo_models/uav_models/p450_D435i/p450_D435i.sdf.jinja`
- Modify: `src/p450/prometheus_gazebo/gazebo_models/uav_models/p450_D435i_mid360/p450_D435i_mid360.sdf.jinja`
- Modify: `src/platform/sim_platform_bringup/config/p450_mid360_tf_contract.yaml`
- Modify: `src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro`
- Modify: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/renderer.py`
- Modify: `src/platform/bunker_sim_runtime/package.xml`
- Test: `tests/test_mid360_runtime_contract.py`
- Test: `tests/test_p450_runtime_contract.py`
- Test: `src/platform/bunker_sim_runtime/test/test_renderer.py`
- Test: `tests/test_ground_manipulator_platform.py`
- Modify: `scripts/smoke_p450_standalone.bash`
- Modify: `scripts/smoke_p450_mid360.bash`
- Modify: `scripts/check_bunker_runtime.py`

**Step 1: Write failing sensor contract tests**

Require:

- `/uav1/livox/lidar` is `sensor_msgs/PointCloud2` with frame `uav1/lidar_link`;
- active P450 runtime models do not publish `/uav1/prometheus/ground_truth`;
- existing D435 topics are directly exposed with no relay nodes;
- BUNKER standalone and combined descriptions expose `ground/imu_link` and `/ground/imu/data` as `sensor_msgs/Imu`;
- `/ground/scan` remains `sensor_msgs/LaserScan`.

**Step 2: Verify RED**

Run the focused tests. Expected failures: Prometheus custom MID360 type, active ground truth, and missing IMU.

**Step 3: Apply minimal sensor changes**

- Select the existing Livox plugin's PointCloud2-with-Livox-fields mode; do not add a cloud relay.
- Remove the active P450 ground-truth publisher; retain Gazebo model state only for SIM test oracles outside common runtime.
- Add one Gazebo IMU sensor/link to both BUNKER rendering paths and publish directly to `/ground/imu/data`.
- Update smoke scripts to inspect `sensor_msgs/PointCloud2`, not `prometheus_msgs/LivoxCustomMsg`.

**Step 4: Build and verify GREEN**

Run:

```bash
/usr/bin/python3 -m unittest -q tests.test_mid360_runtime_contract tests.test_p450_runtime_contract tests.test_ground_manipulator_platform
/usr/bin/python3 src/platform/bunker_sim_runtime/test/test_renderer.py -q
./scripts/with_p450_env.bash catkin_make
```

Expected: all sensor contracts pass and both render paths build.

**Step 5: Commit**

```bash
git add src/p450 src/platform/sim_platform_bringup src/platform/ground_manipulator_runtime src/platform/bunker_sim_runtime scripts/smoke_p450_standalone.bash scripts/smoke_p450_mid360.bash scripts/check_bunker_runtime.py tests/test_mid360_runtime_contract.py tests/test_p450_runtime_contract.py tests/test_ground_manipulator_platform.py
git commit -m "refactor: expose driver-compatible sensor topics"
```

## Task 6: Make navigation a SIM/REAL-common layer

**Files:**

- Modify: `src/ground/bunker_navigation/package.xml`
- Modify: `src/ground/bunker_navigation/CMakeLists.txt`
- Modify: `src/ground/bunker_navigation/config/costmap_common.yaml`
- Modify: `src/ground/bunker_navigation/config/global_costmap.yaml`
- Modify: `src/ground/bunker_navigation/config/local_costmap.yaml`
- Modify: `src/ground/bunker_navigation/config/local_planner.yaml`
- Modify: `src/ground/bunker_navigation/config/move_base.yaml`
- Create: `src/ground/bunker_navigation/launch/ground_navigation.launch`
- Create: `src/ground/bunker_navigation/src/bunker_navigation/stop.py`
- Create: `src/ground/bunker_navigation/scripts/ground_stop_server.py`
- Create: `src/ground/bunker_navigation/test/test_common_navigation.py`
- Modify: `src/ground/bunker_navigation/test/test_configuration.py`

**Step 1: Write failing common-navigation tests**

Require:

- one namespaced move_base action at `/ground/move_base`;
- `map`, `ground/odom`, and `ground/base_link` in both costmaps/local planner;
- direct use of `/ground/scan` and `/ground/odom`;
- move_base output remapped to `/ground/nav_cmd_vel`;
- `/ground/runtime/stop` cancels move_base and publishes zero to `/ground/nav_cmd_vel`;
- the common launch imports no Gazebo package and starts no heading gate or lifecycle node.

**Step 2: Verify RED**

Run:

```bash
/usr/bin/python3 src/ground/bunker_navigation/test/test_common_navigation.py -q
/usr/bin/python3 src/ground/bunker_navigation/test/test_configuration.py -q
```

Expected: FAIL because common launch/stop helper are absent and frames/topics are legacy values.

**Step 3: Implement common navigation**

Create one reusable launch file and a small stop service. Retain legacy demos only as inactive examples; do not launch their Gazebo plugin, heading gate, or lifecycle pieces from the common runtime.

**Step 4: Verify GREEN**

Run the focused tests and catkin build.

**Step 5: Commit**

```bash
git add src/ground/bunker_navigation
git commit -m "feat: add common BUNKER navigation layer"
```

## Task 7: Hide Gazebo bilateral contacts behind `grasp_confirmed`

**Files:**

- Create: `src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/grasp_confirmation.py`
- Create: `src/platform/ground_manipulator_runtime/scripts/gazebo_grasp_confirmation.py`
- Modify: `src/platform/ground_manipulator_runtime/CMakeLists.txt`
- Modify: `src/platform/ground_manipulator_runtime/package.xml`
- Create: `src/platform/ground_manipulator_runtime/test/test_grasp_confirmation.py`
- Modify: `src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch`
- Modify: `tests/test_air_ground_pick_demo.py`

**Step 1: Write failing semantic-adapter tests**

Test a pure evaluator that requires fresh left-pad target contact, fresh right-pad target contact, and a valid closed gripper state before returning true. Structurally require:

- only the SIM adapter imports `gazebo_msgs` and subscribes to Gazebo contacts;
- the public output is `/ground/gripper/grasp_confirmed` as `std_msgs/Bool`;
- the demo orchestrator contains no `ContactsState`, collision-name, or bilateral-contact logic.

**Step 2: Verify RED**

Run focused unit/contract tests; expect failure because the adapter is absent and the demo owns contact semantics.

**Step 3: Implement the SIM adapter**

Move the existing strict bilateral-contact interpretation into the Gazebo-only adapter and publish the minimum backend-independent result. Do not change the physical-contact requirement or add attachment shortcuts.

**Step 4: Verify GREEN**

Run focused tests and catkin build.

**Step 5: Commit**

```bash
git add src/platform/ground_manipulator_runtime src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch tests/test_air_ground_pick_demo.py
git commit -m "refactor: expose backend-neutral grasp confirmation"
```

## Task 8: Refactor the Air-Ground Pick Demo onto common interfaces

**Files:**

- Modify: `src/demos/air_ground_pick_demo/config/demo.yaml`
- Modify: `src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch`
- Modify: `src/demos/air_ground_pick_demo/scripts/run_air_ground_pick_demo.py`
- Modify: `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/approach.py`
- Modify: `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/flight.py`
- Modify: `src/demos/air_ground_pick_demo/CMakeLists.txt`
- Modify: `src/demos/air_ground_pick_demo/package.xml`
- Create: `src/demos/air_ground_pick_demo/test/test_approach.py`
- Create: `src/demos/air_ground_pick_demo/test/test_orchestrator_contract.py`
- Modify: `tests/test_air_ground_pick_demo.py`

**Step 1: Write failing behavior and structural tests**

Require:

- flight operations use `/uav1/runtime/flight` and `FlightCommandAction`;
- ground approach uses `/ground/move_base` and the stop service, never a direct task-owned Twist publisher;
- all target and navigation poses use `map`;
- grasp gating consumes `/ground/gripper/grasp_confirmed`;
- common demo code contains no `gazebo_msgs`, Gazebo services, ground-truth topic, `world` frame, Prometheus command/setup publishers, or SIM-only operation timeout;
- standoff goal geometry places the base on the current-base side of the target and faces the target;
- the existing 16 mission-state names and one-takeoff/one-landing lifecycle are preserved.

**Step 2: Verify RED**

Run:

```bash
/usr/bin/python3 src/demos/air_ground_pick_demo/test/test_approach.py -q
/usr/bin/python3 src/demos/air_ground_pick_demo/test/test_orchestrator_contract.py -q
/usr/bin/python3 -m unittest -q tests.test_air_ground_pick_demo
```

Expected: FAIL on current direct Prometheus, Twist, contacts, and `world` dependencies.

**Step 3: Replace backend calls without changing task intent**

- Use the flight action for takeoff, observation fly-to/hover, and land.
- Compute and send one `MoveBaseGoal` for ground approach in `map`; use the stop service on completion/abort.
- Subscribe to common state/pose/sensor and `grasp_confirmed` topics.
- Keep existing AUBO MoveIt/trajectory and AG95 trajectory interfaces unchanged.
- Keep the exact natural task sequence: one flight, target observation, conditional ground motion, D435 near-field observation, physical grasp, lift, stop/end.

**Step 4: Verify GREEN**

Run focused tests, then the complete fast suite:

```bash
/usr/bin/python3 src/demos/air_ground_pick_demo/test/test_approach.py -q
/usr/bin/python3 src/demos/air_ground_pick_demo/test/test_orchestrator_contract.py -q
/usr/bin/python3 -m unittest -q tests.test_air_ground_pick_demo
/usr/bin/python3 -m unittest -q tests.test_air_ground_pick_demo tests.test_air_ground_platform tests.test_bunker_build_contract tests.test_bunker_shell_contract tests.test_bunker_simple_smoke tests.test_ground_manipulator_platform tests.test_mid360_runtime_contract tests.test_p450_build_contract tests.test_p450_runtime_contract tests.test_realsense_runtime_safety
```

Expected: all unit and static contract tests pass.

**Step 5: Commit**

```bash
git add src/demos/air_ground_pick_demo tests/test_air_ground_pick_demo.py
git commit -m "refactor: run pick demo through common robot interfaces"
```

## Task 9: Update runtime checks and prove interface composition

**Files:**

- Modify: `scripts/check_air_ground_runtime.py`
- Modify: `scripts/check_air_ground_pick_demo.py`
- Modify: `scripts/check_bunker_runtime.py`
- Modify: `scripts/smoke_air_ground_standalone.bash`
- Modify: `scripts/smoke_bunker_standalone.bash`
- Modify: `scripts/smoke_air_ground_pick_demo.bash`
- Modify: `tests/test_runtime_boundary.py`
- Modify: `tests/test_runtime_manifest.py`
- Modify: `tests/test_runtime_source_contract.py`
- Modify: `README.md`

**Step 1: Write/adjust failing runtime-boundary tests**

Require the composed platform to expose:

- flight action plus native P450 state/odom;
- move_base/stop plus official-compatible BUNKER topics;
- shared `map` TF tree with one authority per edge;
- standard RGB-D/MID360/LiDAR/IMU topics;
- existing MoveIt/trajectory interfaces and `grasp_confirmed`;
- no public common-runtime Gazebo ground truth/contact/plugin-control dependency.

Keep Gazebo model/contact inspection in `check_air_ground_pick_demo.py` only as a SIM E2E oracle, not a runtime dependency.

**Step 2: Verify RED, then update checks minimally**

Run the affected tests before and after edits. Update manifests and smoke assertions only for the approved contract; do not add evidence bundles or new lifecycle gates.

**Step 3: Run build and non-E2E validation**

Run:

```bash
./scripts/build_p450_runtime_overlays.bash
/usr/bin/python3 -m unittest -q tests.test_air_ground_pick_demo tests.test_air_ground_platform tests.test_bunker_build_contract tests.test_bunker_shell_contract tests.test_bunker_simple_smoke tests.test_ground_manipulator_platform tests.test_mid360_runtime_contract tests.test_p450_build_contract tests.test_p450_runtime_contract tests.test_realsense_runtime_safety tests.test_sim_to_real_interface_contract
```

Expected: builds and tests pass.

**Step 4: Commit**

```bash
git add scripts tests README.md
git commit -m "test: verify sim-to-real runtime boundaries"
```

## Task 10: Run natural Gazebo E2E and finish the branch

**Files:**

- Modify only if a real failure requires it: files already listed in Tasks 1-9
- Generated and untracked only: the timestamped child created beneath `logs/air_ground_pick_task5_smoke/`

**Step 1: Run standalone platform smokes**

Run serially:

```bash
./scripts/smoke_p450_standalone.bash
./scripts/smoke_bunker_standalone.bash
./scripts/smoke_ground_manipulator_standalone.bash
./scripts/smoke_air_ground_standalone.bash
```

Expected: process health, topics, TF ownership, sensors, controllers, and clean teardown pass.

**Step 2: Run one natural Air-Ground Pick Demo**

Run:

```bash
./scripts/smoke_air_ground_pick_demo.bash
```

Expected:

- exact existing 16-state mission sequence;
- exactly one P450 takeoff and one landing;
- target obtained through normal perception flow;
- ground moves through `move_base` if required and ends stopped;
- AUBO+D435 near-field observation succeeds;
- strict physical grasp becomes `grasp_confirmed` through the SIM adapter;
- target lifts by the existing threshold without teleport or illegal attachment;
- final state is `GROUND_STOPPED` and teardown is clean.

**Step 3: Inspect the generated summary**

Open the newest `logs/air_ground_pick_task5_smoke/*/summary.json` and record its path and primary measurements. Do not create an evidence framework around it.

**Step 4: Run final regression from a clean process state**

Run:

```bash
git status --short
/usr/bin/python3 -m unittest -q tests.test_air_ground_pick_demo tests.test_air_ground_platform tests.test_bunker_build_contract tests.test_bunker_shell_contract tests.test_bunker_simple_smoke tests.test_ground_manipulator_platform tests.test_mid360_runtime_contract tests.test_p450_build_contract tests.test_p450_runtime_contract tests.test_realsense_runtime_safety tests.test_sim_to_real_interface_contract
```

Expected: tests pass; only intended source/doc changes or ignored log artifacts exist.

**Step 5: Review and final commit**

Use the required verification and code-review skills. Fix only concrete correctness/scope issues. Then commit any integration fixes:

```bash
git add src/platform src/ground/bunker_navigation src/demos/air_ground_pick_demo src/vendor/bunker_msgs scripts tests README.md
git commit -m "feat: complete sim-to-real robot interface layer"
```

## Final delivery checklist

- List the exact SIM/REAL-common action, topic, service, and TF contracts.
- List remaining simulator-specific coupling and confirm it is confined to SIM launch/adapters/test oracles.
- Identify the future P450 replacement boundary: physical PX4/MAVROS/Prometheus/p450_experiment plus a real localization adapter; the thin facade remains.
- Identify the future BUNKER replacement boundary: official bunker_base/ugv_sdk/CAN plus a real localization adapter and sensor drivers; move_base remains.
- Identify unchanged upper code: Air-Ground task sequencing, future AGENT/NBV/RM4D clients of common runtime, common navigation, MoveIt, and manipulation trajectory clients.
- Report the exact final test/build/E2E commands and newest E2E summary path.
- Confirm no files under `P450-PAPER` outside `SIM/` were modified.
