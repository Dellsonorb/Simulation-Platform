# Ground Manipulator A Design

## Purpose

Ground Manipulator A is the reusable ground robot runtime for Simulation
Platform V1.0. It runs one physically connected Gazebo model containing the
BUNKER base, AUBO i5 arm, AG95 gripper, wrist-mounted D435, and 2D LiDAR. It is
worldless so a standalone wrapper or the shared air-ground world can own the
single Gazebo process.

The runtime contains robot models, sensors, TF, guarded base velocity,
ros_control controllers, and optional MoveIt execution. It does not contain a
brick task, mission state machine, handoff protocol, benchmark, research
method, provenance gate, or lifecycle supervisor.

## Architecture

Create `ground_manipulator_runtime` under `src/platform`. Its renderer expands
the SIM-local vendor xacros, canonicalizes the exported BUNKER names, prefixes
link frames with `ground/`, fixes the non-actuated BUNKER wheel joints, replaces
the high-cost BUNKER mesh collision with the validated box collision, and
rejects an unexpected plugin or malformed robot tree.

The resulting Gazebo model is named `ground_robot`. Link frame names are
explicitly prefixed (`ground/base_link`, `ground/aubo_i5_base_link`, and so on),
while controller joint names remain the standard AUBO/AG95 names expected by
MoveIt. This avoids TF-prefix ambiguity without changing controller APIs.

The model owns these simulation plugins:

- the existing SIM-owned BUNKER planar movement plugin;
- Gazebo ROS 2D LiDAR;
- `gazebo_ros_control` in `/ground`;
- the upstream AG95 mimic mechanism;
- Gazebo ROS color and depth cameras under `/ground/d435`.

The runtime launch starts no Gazebo process. It publishes the model, launches
the existing BUNKER velocity guard, spawns `ground_robot`, starts the ground
controller manager clients and robot state publisher, and anchors
`ground/odom` to `world`. A standalone wrapper owns Gazebo for isolated tests.

## Interfaces

- Base input: `/ground/cmd_vel`
- Guarded base input: `/ground/cmd_vel_safe`
- Base odometry: `/ground/odom`
- 2D LiDAR: `/ground/scan`
- Arm action: `/ground/arm_controller/follow_joint_trajectory`
- Gripper action: `/ground/gripper_controller/follow_joint_trajectory`
- Joint state: `/ground/joint_states`
- D435 color: `/ground/d435/color/image_raw`
- D435 depth: `/ground/d435/depth/image_raw`
- TF root: `world -> ground/odom -> ground/base_link`
- Manipulator TF tip: `ground/gripper_tcp_link`

MoveIt runs as an optional planning/execution layer. It reads the same rendered
robot model, subscribes to `/ground/joint_states`, plans for the standard
`manipulator` and `gripper` groups, and sends trajectories only to the two
namespaced real controllers.

## Runtime checks

The isolated smoke must prove:

1. one finite `ground_robot` Gazebo model exists;
2. arm and gripper controllers are running;
3. all expected joints have finite, current states;
4. a bounded arm trajectory changes a real Gazebo joint and reaches tolerance;
5. open and close gripper trajectories move the real master joint;
6. D435 color and depth frames advance with correct dimensions and frame IDs;
7. the complete world-to-camera and world-to-gripper TF chains exist;
8. BUNKER LiDAR and bounded planar motion remain correct;
9. MoveIt plans and executes a bounded arm trajectory through the controller;
10. teardown leaves no ROS, Gazebo, or controller process from the run.

After the isolated runtime is stable, the shared launch replaces the
BUNKER-only model with `ground_robot`; it does not add a second Gazebo or an
overlaid arm model.

## Failure behavior

The launch fails when rendering or model spawning fails. Controller spawners
are required. Runtime checks reject missing or non-finite joint data, stale
camera data, missing TF, failed actions, excessive final joint error, failed
MoveIt execution, or unexpected launcher exit. Base commands remain bounded by
the velocity guard and its command timeout.

## Deliberate exclusions

This stage does not load brick models, guarded attachment, grasp orchestration,
navigation, aerial observation logic, benchmark scenarios, paper evidence,
Pilot/T2/Formal, RBP/M5/Task-aware, RL, world models, or LLM/VLM logic. The
minimal Air-Ground Pick Demo is a later integration step after this runtime is
independently stable.
