# Ground execution reliability — development checkpoint

2026-09-09, `feature/fix-ground-execution-reliability`, based on the existing
`feature/fix-ground-latch-namespace` correction (`ece36ec`). This is a shared
SIM execution revision, not an RM4D or active-perception policy change.
Do not infer a complete retrieval regression pass from the navigation fixes.
The bounded online records and full report live in AGENT:
`outputs/development/ground-execution-batch/` and
`docs/GROUND_EXECUTION_BATCH_RESULTS.md`.

## Diagnosed navigation corrections

1. Install the existing public XY latch namespace correction. It alone still
   times out on the original Easy goal in launch01.
2. Read completed-physics linear/angular velocity **before** applying the next
   command. Previously odometry reported an unachieved command as feedback.
   Launch02 confirms the feedback correction but still times out.
3. Preserve the base collision's already declared zero-friction surface through
   URDF→SDFormat conversion. `base_link_collision` on `ground/base_link` lost
   that extension; `ground/base_link_collision` retains it. A real SDFormat9
   conversion test compares geometry, inertias, 17 joints, and all other
   collision surfaces. This does not remove a collision or change the declared
   friction value. Launch05 recovers command tracking but still times out.
4. Disable periodic same-goal global replans (`planner_frequency: 0`); replan on
   new goals/control failure. DWA resets its XY latch on every `setPlan`, so the
   old 2 Hz periodic updates interrupted final rotation. Local costmaps,
   collision checking, 15 Hz control, velocities, acceleration, timeout, and
   goal tolerances remain active/unchanged. Launch06 returns MoveBase success,
   but its actual XY error is 98.66 mm: this is **not** accurate arrival.
5. Command the correct rigid-body reference point:
   `v_CoG = v_base_link + omega × (p_CoG − p_base_link)`, all in world axes.
   Gazebo ODE's velocity setter addresses the center of mass, whereas the ROS
   command addresses the link origin. Converted base CoG relative to that
   origin is approximately `(0.024387, 0.003131, -0.102779)` m; it is read from
   the live model, not hard coded. A compiled numerical test checks transport.
6. After stopping, check fresh public pose against the original 0.06 m XY and
   0.08 rad yaw limits. Do not accept a stale XY latch as actual arrival.
   Launch07 reaches 59.414 mm / 0.068291 rad and remains parked during the arm
   segment. This is one successful local navigation check, near the tolerance
   boundary, not a statistical reliability claim.

Relevant upstream implementations:
[DWA setPlan](https://github.com/ros-planning/navigation/blob/1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp),
[Gazebo ODELink](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/ode/ODELink.cc),
[SDFormat9 URDF conversion](https://github.com/gazebosim/sdformat/blob/sdf9/src/parser_urdf.cc).

## Camera-centered observation v1

`ground_observation_mode: camera_centered_v1` separates observation planning
from grasp IK. Legacy mode remains available for historical reproduction.
After a short check for an already useful fresh view, generate six fixed views:
target-top optical depths 0.45/0.55/0.35 m × two optical rolls. Compose the full
measured TCP↔optical transform; use CameraInfo to test all eight known cuboid
corners. Predicted FOV is not treated as proof of absence of occlusion.
Each observation plan remains collision aware. Only real fresh RGB-D can
provide the final refined pose; execution failure terminates the segment.

A fresh measured red surface may supply an **XY aiming cue only**. It is not
a cuboid-center estimate, cannot set grasp height, and cannot confirm a target.
Ground-only pose estimation now requires an unclipped, near-horizontal measured
top (within 15 degrees), with measured spans consistent with 90–110% of the
known 0.240 × 0.053 m top. The existing 15 mm top band and center-height gate
remain in use. Measured top height, not nominal ground/target height, determines
the center. The span check is conservative and can reject partial/occluded
tops; it is not a general object reconstruction algorithm.

The original aerial observer defaults are unchanged. Offline replay rejects
the old Easy clipped vertical face instead of accepting its erroneous center
height. New Easy and Moderate runs obtain fresh valid Ground refinement.
This addresses observation coupling/partial-surface errors, not all grasp
feasibility or occlusion limits.

## Verification and remaining failure

Fresh targeted tests: 89 SIM observer/demo/platform/conversion/twist tests,
7 approach tests, and 7 navigation configuration tests pass. Catkin p450-clean
build/install of the changed runtime, navigation, and demo packages passes.
The actual online build includes both the origin transport and fresh-arrival
gate in launch07/08.

The optional Ground-only physical checker omits nonexistent flight stages,
not real robot conditions: fresh refinement, controller success, grasp
confirmation, and physical target lift remain required. It never sends target
GT to the algorithm. AGENT's optional collision-disabled IK query is a
post-failure, non-executing diagnostic only; production plans remain collision
aware.

Easy launches03/06/07 reach refined pregrasp, descend, and close, but lift fails
the unchanged wrist-3 stopped-velocity condition. Position and velocity traces
need further controller/physics diagnosis. No joint gain, velocity tolerance,
goal settling timeout, grasp acceptance, or lift criterion was relaxed.
Moderate's final detailed outcome is recorded in the AGENT batch report.

Keep this branch/PR as a development checkpoint. No full-chain success or
formal-readiness claim; do not merge solely because the unit tests pass.
