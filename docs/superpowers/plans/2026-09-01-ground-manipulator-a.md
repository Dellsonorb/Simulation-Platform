# Ground Manipulator A Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and verify a worldless BUNKER + AUBO i5 + AG95 + D435 runtime with real Gazebo controllers, namespaced TF/sensors, and MoveIt execution.

**Architecture:** Add a focused `ground_manipulator_runtime` platform package. A small renderer composes SIM-local vendor xacros into one validated robot with explicit `ground/` link frames; its launch owns robot interfaces but not Gazebo. Add a clean MoveIt entrypoint, isolated behavior checker, and finally replace the BUNKER-only model in the existing shared world.

**Tech Stack:** ROS Noetic, catkin, Gazebo 11, xacro/URDF, gazebo_ros_control, ros_control trajectory controllers, MoveIt 1, Python 3 unittest/rospy/actionlib.

---

### Task 1: Define the clean package and renderer transformation

**Files:**
- Create: `src/platform/ground_manipulator_runtime/CMakeLists.txt`
- Create: `src/platform/ground_manipulator_runtime/package.xml`
- Create: `src/platform/ground_manipulator_runtime/setup.py`
- Create: `src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/__init__.py`
- Create: `src/platform/ground_manipulator_runtime/src/ground_manipulator_runtime/renderer.py`
- Create: `src/platform/ground_manipulator_runtime/scripts/render_ground_robot.py`
- Create: `tests/test_ground_manipulator_platform.py`

- [ ] **Step 1: Write the failing package/transformation contract**

  Assert that the package declares the BUNKER, AUBO, AG95, Gazebo control,
  camera and MoveIt dependencies. Feed a minimal in-memory robot fixture to the
  renderer transformation and assert explicit `ground/` link names, canonical
  joint names, updated URDF/SDF/Gazebo references, fixed BUNKER wheel joints,
  and the validated BUNKER box collision.

- [ ] **Step 2: Run the contract and verify RED**

  Run: `/usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform`

  Expected: failure because the package and transformation do not exist.

- [ ] **Step 3: Implement the minimal renderer**

  Implement a pure XML transformation that canonicalizes declarations and
  references, prefixes only link names with `ground/`, makes wheel joints
  fixed, and installs the validated base collision. Add the CLI/package shell;
  complete xacro expansion and full model validation in Task 2.

- [ ] **Step 4: Run focused tests and commit**

  Run: `/usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform`

  Expected: package and pure transformation tests pass.

  Commit: `feat: define ground manipulator runtime model`

### Task 2: Add the composite model, controllers and worldless launch

**Files:**
- Create: `src/platform/ground_manipulator_runtime/urdf/ground_robot.urdf.xacro`
- Create: `src/platform/ground_manipulator_runtime/urdf/d435.xacro`
- Create: `src/platform/ground_manipulator_runtime/config/controllers.yaml`
- Create: `src/platform/ground_manipulator_runtime/launch/ground_robot_runtime.launch`
- Create: `src/platform/ground_manipulator_runtime/launch/ground_robot_standalone.launch`
- Create: `src/platform/ground_manipulator_runtime/worlds/ground_robot.world`
- Modify: `tests/test_ground_manipulator_platform.py`

- [ ] **Step 1: Write the failing launch/model contracts**

  Assert the complete renderer output has one root link, required arm/gripper
  joints and transmissions, only the required runtime plugins, D435 and LiDAR
  sensors, and no brick/handoff tokens. Also assert one spawn node for
  `ground_robot`, no Gazebo include in the runtime, one Gazebo include in the
  standalone wrapper, `/ground` controller and sensor interfaces, required
  velocity guard, three controllers, and explicit world-to-odom TF.

- [ ] **Step 2: Run tests and verify RED**

  Run: `/usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform`

  Expected: missing model, controller, launch and world files.

- [ ] **Step 3: Implement the minimal composite runtime**

  Complete the renderer's xacro expansion and runtime-tree validation. Include
  the SIM-local BUNKER, AUBO and AG95 xacros; mount AUBO on the base,
  AG95 and a gripper TCP on the wrist, the D435 beside the gripper, the 2D
  LiDAR on the base, and configure the existing planar plugin plus
  `gazebo_ros_control`. Load bounded position trajectory controllers and spawn
  them under `/ground`.

- [ ] **Step 4: Validate URDF, launch resolution and build**

  Run:

  ```bash
  /usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform
  catkin build ground_manipulator_runtime --no-deps
  source install/p450-clean/setup.bash
  check_urdf /tmp/ground_robot.urdf
  roslaunch --files ground_manipulator_runtime ground_robot_standalone.launch gui:=false
  ```

  Expected: tests/build/URDF/launch resolution pass.

  Commit: `feat: launch ground manipulator in Gazebo`

### Task 3: Add the clean MoveIt execution entrypoint

**Files:**
- Create: `src/ground/bunker_aubo_moveit_config/config/ground_robot.srdf`
- Create: `src/ground/bunker_aubo_moveit_config/config/ground_controllers.yaml`
- Create: `src/ground/bunker_aubo_moveit_config/launch/ground_move_group.launch`
- Modify: `src/ground/bunker_aubo_moveit_config/package.xml`
- Modify: `src/platform/ground_manipulator_runtime/launch/ground_robot_runtime.launch`
- Modify: `tests/test_ground_manipulator_platform.py`

- [ ] **Step 1: Write the failing MoveIt contract**

  Assert the SRDF uses `ground/aubo_i5_base_link` through
  `ground/gripper_tcp_link`, controller actions are `/ground/arm_controller`
  and `/ground/gripper_controller`, joint states remap to
  `/ground/joint_states`, and MoveIt is optional in the worldless runtime.

- [ ] **Step 2: Run tests and verify RED**

  Run: `/usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform`

  Expected: missing clean MoveIt files and include.

- [ ] **Step 3: Implement the MoveIt configuration**

  Load the clean SRDF, existing kinematics/joint-limit/OMPL settings, the
  namespaced controller list, and a root `move_group` that reads the rendered
  `/robot_description` and consumes `/ground/joint_states`.

- [ ] **Step 4: Run static and package tests**

  Run:

  ```bash
  /usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform
  catkin run_tests bunker_aubo_moveit_config --no-deps
  ```

  Expected: all tests pass.

  Commit: `feat: execute ground arm plans through MoveIt`

### Task 4: Prove the isolated runtime behavior

**Files:**
- Create: `scripts/check_ground_manipulator_runtime.py`
- Create: `scripts/smoke_ground_manipulator_standalone.bash`
- Modify: `tests/test_ground_manipulator_platform.py`

- [ ] **Step 1: Write failing checker and smoke contracts**

  Unit-test finite/fresh joint and image summaries, bounded action goals,
  final-position tolerances, TF pairs, one-subscription camera behavior,
  timeout handling, teardown-before-PASS, and compact JSON output.

- [ ] **Step 2: Run tests and verify RED**

  Run: `/usr/bin/python3 -B -m unittest tests.test_ground_manipulator_platform`

  Expected: missing checker and smoke behavior.

- [ ] **Step 3: Implement and run the real smoke**

  The checker waits for `ground_robot`, running controllers and current joint
  data; sends bounded arm and gripper trajectories; verifies current D435 and
  LiDAR data plus TF; commands bounded base motion; then asks MoveIt to plan and
  execute a small arm move. The shell wrapper waits for simulated time, bounds
  total runtime, records one log and JSON summary, and reports PASS only after
  roslaunch exits cleanly.

  Run: `scripts/smoke_ground_manipulator_standalone.bash --gui false`

  Expected: two consecutive PASS runs with no residual ROS/Gazebo processes.

- [ ] **Step 4: Run regression and commit**

  Run:

  ```bash
  /usr/bin/python3 -B -m unittest discover -s tests -p 'test_*.py'
  catkin run_tests ground_manipulator_runtime bunker_aubo_moveit_config --no-deps
  git diff --check
  ```

  Expected: all tests pass.

  Commit: `test: prove ground manipulator runtime`

### Task 5: Integrate the composite ground robot into the shared world

**Files:**
- Modify: `src/platform/sim_platform_bringup/launch/air_ground_standalone.launch`
- Modify: `src/platform/sim_platform_bringup/package.xml`
- Modify: `scripts/check_air_ground_runtime.py`
- Modify: `tests/test_air_ground_platform.py`

- [ ] **Step 1: Write the failing shared-world contract**

  Require the ground manipulator worldless launch instead of the BUNKER-only
  launch, model `ground_robot`, arm/gripper controller health, D435 frames and
  manipulator TF while retaining every P450 and BUNKER behavior check.

- [ ] **Step 2: Run tests and verify RED**

  Run: `/usr/bin/python3 -B -m unittest tests.test_air_ground_platform`

  Expected: old BUNKER-only include/model assertions fail.

- [ ] **Step 3: Replace only the ground runtime include**

  Keep exactly one Gazebo world owner and the existing P450 include. Forward
  the same ground pose arguments to `ground_robot_runtime.launch`; do not load
  brick, navigation or demo orchestration.

- [ ] **Step 4: Run shared smoke and regression**

  Run:

  ```bash
  P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
    scripts/smoke_air_ground_standalone.bash --gui false
  /usr/bin/python3 -B -m unittest discover -s tests -p 'test_*.py'
  git diff --check
  ```

  Expected: two consecutive shared-world PASS runs and a clean external
  P450-PAPER status.

  Commit: `feat: add manipulation to the shared air-ground world`
