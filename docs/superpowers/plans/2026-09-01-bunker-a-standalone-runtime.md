# BUNKER-A V1-simple implementation plan

This plan supersedes the earlier evidence-oriented BUNKER-A plan.

## 1. Preserve the real robot runtime

- Keep the installed BUNKER model, namespace, velocity guard, 2D LiDAR, odometry,
  TF chain, standalone world, and reusable runtime launch.
- Keep the base collision/friction correction that makes commanded rotation
  agree with the planar motion plugin.
- Replace the system planar plugin whose callback thread crashes on model
  unload with a small SIM-owned velocity/odometry/TF plugin.

## 2. Remove paper-style runtime infrastructure

- Remove the live evidence contract, provenance/hashing pipeline, probe schema,
  and custom process identity supervisor.
- Replace them with one ROS behavior checker and one short launch/cleanup script.
- Keep only a human-readable runtime log and a compact JSON result.

## 3. Verify BUNKER-A

- Run Python and catkin tests.
- Rebuild the two BUNKER packages into `install/p450-clean`.
- Validate the installed package closure and plugins.
- Run the standalone smoke twice and inspect motion, scan, TF, and shutdown.
- Confirm the external P450-PAPER repositories remain unchanged.

## 4. Continue Simulation Platform V1.0

- Include `bunker_runtime.launch` in the shared P450 world.
- Then add AUBO i5 + AG95 + D435 and MoveIt execution.
- Build the minimal observe → ground move if needed → near-field observe →
  contact grasp → lift sequence only after each robot subsystem is stable alone.
