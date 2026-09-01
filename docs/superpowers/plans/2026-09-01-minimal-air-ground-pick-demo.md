# Minimal Air-Ground Pick Demo Implementation Plan

> Scope: SIM-only platform demonstration. No benchmark, research algorithm,
> provenance system, adversarial gate, or complex lifecycle framework.

## Goal

Run one natural Gazebo sequence:

1. P450 arms, takes off once, moves to one observation pose, and observes a
   physical red target through its D435.
2. The aerial sensor pose is handed to Ground Robot through a normal ROS pose
   topic.
3. BUNKER moves only as far as needed to enter the verified AUBO work range.
4. AUBO moves the wrist-mounted D435 to a near-field observation pose and
   re-estimates the target.
5. MoveIt and the real Gazebo controllers execute pre-grasp, approach, AG95
   close, and lift.
6. The run ends after verified target lift.

## Architecture

Create one focused `air_ground_pick_demo` package under `src/demos/`.

- `red_target_observer.py` is a reusable, camera-configured RGB-D node. It
  extracts one red cuboid from synchronized RGB, depth, and calibration data,
  transforms the measured points through TF, and publishes a fresh
  `PoseStamped`. It has no Gazebo model-state dependency.
- `air_ground_pick_demo.py` is a small sequential orchestrator. It speaks only
  the existing Prometheus, `/ground/cmd_vel`, controller action, MoveIt,
  sensor-pose, TF, and contact interfaces.
- `pick_target.sdf` is a dynamic, inertial, collision-enabled red cuboid whose
  short span fits the real AG95 opening. A Gazebo contact sensor exposes actual
  collision contacts for validation.
- `air_ground_pick_demo.launch` owns no Gazebo instance. It includes the
  existing shared Air/Ground runtime, starts the optional MoveIt layer, spawns
  the target, and starts the two observer nodes and orchestrator.
- `check_air_ground_pick_demo.py` is a bounded E2E checker for status,
  perception freshness, physical contacts, and measured target lift. Gazebo
  model pose may be used by this checker as an external test oracle, never by
  the robot behavior or perception nodes.

The nominal layout is intentionally simple:

- target center: `(2.0, 0.0, 0.0575)`, side-up dimensions
  `0.240 x 0.053 x 0.115 m`;
- Ground Robot: `(3.5, 0.0, 0.36)`, yaw `pi`, initially facing the target;
- P450: starts at the existing origin and flies to one unobstructed pose on
  the target's negative-X side.

## Physical constraints

- No pose teleport or set-model-state call.
- Neither observer may read `/gazebo/model_states` or a target ground-truth
  topic.
- Ground approach is odometry/TF closed-loop and bounded by the current 2-D
  LiDAR. The target itself remains protected by a nonzero work standoff.
- Arm motion is planned/executed by MoveIt and the real namespaced trajectory
  controllers.
- Grasp requires the real AG95 joint and bilateral target/pad contact.
- First implementation relies on Gazebo contact/friction only. If repeated
  testing proves that ODE cannot retain the already contact-qualified object,
  a small automatic grasp stabilizer may be added only after simultaneous
  bilateral pad contact, target bracketing, gripper closure, and bounded
  penetration. It must detach on opening and cannot expose an arbitrary attach
  service.

## Task 1: Define the package, target, and pure perception geometry

Files:

- Create `src/demos/air_ground_pick_demo/CMakeLists.txt`
- Create `src/demos/air_ground_pick_demo/package.xml`
- Create `src/demos/air_ground_pick_demo/setup.py`
- Create `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/__init__.py`
- Create `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/perception.py`
- Create `src/demos/air_ground_pick_demo/models/pick_target/model.sdf`
- Create `src/demos/air_ground_pick_demo/models/pick_target/model.config`
- Create `tests/test_air_ground_pick_demo.py`

Steps:

1. Write failing contracts for one dynamic target with valid inertia,
   collision, red material, high but finite friction, and a contact sensor.
2. Write pure tests for RGB red-mask selection, calibrated back-projection,
   finite-point rejection, target centroid/yaw, and bounded temporal fusion.
3. Implement only the tested geometry and package shell.
4. Build and commit `feat: define minimal pick target perception`.

## Task 2: Add independent aerial and near-field observers

Files:

- Create `src/demos/air_ground_pick_demo/scripts/red_target_observer.py`
- Create `src/demos/air_ground_pick_demo/config/air_observer.yaml`
- Create `src/demos/air_ground_pick_demo/config/ground_observer.yaml`
- Create `src/demos/air_ground_pick_demo/launch/target_observers.launch`
- Modify `tests/test_air_ground_pick_demo.py`

Steps:

1. Contract two independent nodes with explicit camera topics, optical frames,
   output topics, current timestamps, calibration checks, and no GT inputs.
2. Implement synchronized RGB/depth/camera-info processing and TF transforms.
3. Run a sensor-only Gazebo probe for the aerial observer at its single view.
4. Move AUBO to the inherited observation joint pose and run the same probe
   for the ground observer.
5. Tune only thresholds and poses required by the real images, then commit
   `feat: observe pick target from air and ground`.

## Task 3: Add one-shot P450 flight and closed-loop Ground approach

Files:

- Create `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/flight.py`
- Create `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/approach.py`
- Create `src/demos/air_ground_pick_demo/scripts/air_ground_pick_demo.py`
- Create `src/demos/air_ground_pick_demo/config/demo.yaml`
- Modify `tests/test_air_ground_pick_demo.py`

Steps:

1. Unit-test the minimal Prometheus sequence: preflight, arm, command control,
   one takeoff, one view, fresh observation, and land/hold behavior.
2. Unit-test the approach controller: target transform, heading error,
   standoff, velocity bounds, scan safety, stop, and no-motion case.
3. Implement the sequential state publisher and the flight/approach phases.
4. Live-test through `AIR_HANDOFF` and `GROUND_STOPPED`; commit
   `feat: hand aerial observation to ground approach`.

## Task 4: Execute near-field grasp and Lift

Files:

- Create `src/demos/air_ground_pick_demo/src/air_ground_pick_demo/grasp.py`
- Extend `src/demos/air_ground_pick_demo/scripts/air_ground_pick_demo.py`
- Create `src/demos/air_ground_pick_demo/launch/air_ground_pick_demo.launch`
- Modify `tests/test_air_ground_pick_demo.py`

Steps:

1. Unit-test target feasibility against AG95 opening and top-down pre-grasp,
   grasp, and lift pose generation.
2. Move to the known camera observation joint configuration through the real
   arm controller and require a fresh near-field sensor pose.
3. Plan/execute pre-grasp with MoveIt, execute a bounded Cartesian approach,
   close AG95, require bilateral contact, and execute a bounded lift.
4. Verify friction-only retention over a short hold. Add no attachment if it
   succeeds repeatedly.
5. Commit `feat: grasp and lift physical pick target`.

## Task 5: Add bounded E2E smoke and repeatability

Files:

- Create `scripts/check_air_ground_pick_demo.py`
- Create `scripts/smoke_air_ground_pick_demo.bash`
- Modify `tests/test_air_ground_pick_demo.py`

Steps:

1. Require ordered status transitions, fresh aerial and ground observations,
   one flight cycle, bounded Ground motion, successful controller execution,
   bilateral physical contacts, and at least `0.10 m` target lift.
2. Keep the wrapper on isolated ROS/Gazebo ports with bounded teardown.
3. Run two consecutive E2E PASS runs, relevant catkin tests, maintained Python
   tests, `git diff --check`, and residual-process checks.
4. Commit `test: prove minimal air-ground pick demo`.

## Explicit non-goals

- RBP, RM4D, Task-aware, RL, World Model, LLM/VLM.
- M1/M5/Pilot/T2/Formal benchmark infrastructure.
- Multi-view search, adversarial scenarios, lifecycle barrier frameworks.
- Artifact locks, provenance manifests, sealed bundles, evidence pipelines.
- Teleport, GT-based target input, fake controllers, or arbitrary attachment.
