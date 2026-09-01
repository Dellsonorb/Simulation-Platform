# BUNKER-A standalone runtime — V1-simple design

Status: approved implementation baseline for Simulation Platform V1.0.

## Purpose

BUNKER-A is the smallest ground-robot runtime that can later be included in
the shared P450 + BUNKER + AUBO/AG95 world. It proves ordinary robot behavior,
not a paper benchmark or an evidence pipeline.

## Runtime boundary

The actual platform is the pair of launch files in `bunker_sim_runtime`:

- `bunker_runtime.launch` owns the namespaced robot, TF prefix, velocity guard,
  model spawn, robot-state publisher, and `world -> ground/odom` anchor.
- `bunker_standalone.launch` adds Gazebo and the small standalone world.

The same `bunker_runtime.launch` must be reusable from a future shared-world
launch without starting a second Gazebo server.

## Required behavior

- The installed `bunker_description` and `bunker_sim_runtime` packages are the
  only BUNKER package sources used at runtime.
- Gazebo contains one `bunker` model with real collision geometry.
- `/ground/cmd_vel` passes through the velocity guard to the planar plugin.
- The planar plugin is SIM-owned, publishes odometry/TF, applies model linear
  and angular velocity without pose teleportation, and joins its ROS callback
  thread before model unload.
- `/ground/odom` advances during commanded motion and stops after a zero command.
- `/ground/scan` is live, timestamped, and reports the world obstacle.
- `world -> ground/base_link -> ground/lidar_2d_link` is connected.
- A forward command produces forward travel; a yaw command produces rotation
  with small planar drift.
- Shutdown is signal-driven through roslaunch; no teleport or attachment helper
  is part of this runtime.

## Developer helpers

- `with_bunker_env.bash` gives a run its own ROS/Gazebo/cache directories and
  sources the installed workspace.
- `validate_bunker_install.bash` checks packages, URDF generation, launch
  resolution, and plugin dependencies.
- `check_bunker_runtime.py` checks public ROS/Gazebo behavior.
- `smoke_bunker_standalone.bash` starts one roslaunch process group, runs the
  checker, writes `runtime.log` plus a small `summary.json`, and shuts down.

These helpers intentionally do not implement hashes, provenance capture,
immutable artifacts, lifecycle barriers, PID identity protocols, or benchmark
scoring. Git history and normal test output are sufficient for V1 development.

## Acceptance

Source/unit tests, installed-package validation, and two consecutive standalone
smoke runs must pass. The historical `P450-PAPER` repositories are checked
read-only from the development session, not embedded in the robot runtime.
