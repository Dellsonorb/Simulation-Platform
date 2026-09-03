# Deterministic RM4D to Simulation Platform Integration Design

**Status:** Approved and frozen for implementation on 2026-09-03

## Objective

Integrate the read-only `RM4D-AUBO baseline v1` with the existing natural
Air-Ground Pick Demo through the shortest backend-neutral data path:

```text
P450 RGB-D Brick observation
  -> exact Brick pose in map
  -> existing side-up exact grasp TCP
  -> frozen RM4D BasePlacementAPI
  -> unchanged ranked BUNKER poses
  -> /ground/move_base
  -> target-facing AUBO+D435 near-field refinement
  -> existing exact MoveIt/AG95 grasp
  -> Lift
```

RM4D remains an external algorithm dependency. No RM4D algorithm, map, model,
threshold, ranking code, or generated artifact is copied into `SIM/`.

## Frozen external dependency

The only accepted RM4D source revision is:

- repository: `Dellsonorb/RM4D_auboi5`
- tag: `rm4d-aubo-baseline-v1`
- commit: `e9d431299053f38a4a4319aed3dfeccc261b9fac`
- map:
  `runs/formal-10m/data/rm4d_aubo_i5_joint_42/10000000/rmap.npy`
- map SHA-256:
  `77279fafaf61d5c92cf303a644cd9613457c85e0d197c66c4487c30d135db3b4`

The release branch is not an authority because it can move. Launch requires an
external detached checkout at the exact commit plus an explicit RM4D Python
interpreter and map path. The adapter checks the source commit at startup;
`BasePlacementAPI.from_files(...)` retains responsibility for checking the
map hash. The adapter does not clone, update, or modify the external checkout.

## Audited geometry and frame agreement

The current SIM chain already provides the required sensor-derived input:

- `/air_observer/target_pose` is estimated from synchronized P450 RGB-D data.
- The observer transforms the point cloud to `map` through TF.
- The estimate contains Brick center XYZ and its pi-periodic long-axis yaw.
- No Gazebo model state or ground-truth pose participates in this path.

The existing `generate_top_down_grasp(...)` helper remains the single side-up
grasp definition. For the current Brick it uses size
`[0.240, 0.053, 0.115]` m and produces the exact physical
`ground/gripper_tcp_link` pose. It preserves the current pregrasp height,
finger-pad offset, contact overlap, support clearance, and lift height.

The frozen RM4D geometry matches the executed robot model:

```text
T_ground/base_link_ground/aubo_i5_base_link = Trans(0.15, 0, 0.122)

T_ground/wrist_3_link_ground/gripper_tcp_link =
[[0, 1, 0, 0],
 [0, 0, 1, 0],
 [1, 0, 0, 0.2],
 [0, 0, 0, 1]]
```

The navigation footprint plus padding produces the same frozen RM4D rectangle:

```text
[[-0.52, -0.39], [-0.52, 0.39], [0.52, 0.39], [0.52, -0.39]] m
```

All public integration data is in `map`. Gazebo `world` remains private to the
SIM localization adapter and is neither accepted nor published by the RM4D
integration package.

## Adapter architecture

Create one small ROS package, `rm4d_sim_integration`. Its ROS node runs under
the external frozen RM4D Python 3.10 interpreter. That environment has been
verified to import both `BasePlacementAPI` and ROS Noetic Python modules, so no
second JSON worker protocol is needed.

The node constructs one `BasePlacementAPI` at startup and reuses it for all
queries. It closes the API on shutdown. Its only responsibilities are:

1. validate the external checkout and configured paths;
2. accept an exact map-frame grasp TCP through ROS;
3. read the current BUNKER SE(2) pose from `map -> ground/base_link`;
4. convert ROS values to the frozen Python request dictionary;
5. apply the one frozen numerical regularization described below;
6. call `BasePlacementAPI.plan(...)` without modifying its result order;
7. convert candidates back to ROS poses and visualization markers.

The service is `/rm4d/plan_base_placement`:

```text
request:
  geometry_msgs/PoseStamped grasp_tcp
  string grasp_id
  uint32 top_k

response:
  bool success
  string status
  geometry_msgs/PoseArray candidates
  string[] candidate_ids
  float64[] scores
  string message
```

`success` means the adapter completed the external API call. A valid API result
with no candidates uses `success=true`, `status=no_feasible_candidate`, and an
empty candidate array. Transport, frame, dependency, or API exceptions use
`success=false` and a concise error message.

The candidate array order is the frozen RM4D order. The adapter never
re-scores, re-sorts, samples, filters, or replaces candidates. It does not send
RM4D's IK joint configuration to the robot controller; the real execution
authority remains MoveIt and the existing controllers.

## Frozen deterministic numerical regularization

The exact side-up top-down orientation puts RM4D's internal reachability-map
polar angle on the `acos(-1)` floating-point boundary. At zero perturbation the
frozen implementation can produce a value just below `-1` and raise instead of
returning a result.

Only the in-memory quaternion passed to the frozen API is regularized:

```text
R_rm4d_query = R_exact_grasp_tcp * Ry(+1e-6 rad)
p_rm4d_query = p_exact_grasp_tcp
```

This is a fixed integration-layer numerical regularization, not an algorithm
optimization. The value and axis are frozen. There is no runtime epsilon
selection, retry sweep, dynamic clipping, or modification of RM4D core.

The exact, unperturbed pose remains authoritative for:

- `/rm4d/grasp_tcp` publication and RViz display;
- aerial-derived observation/pregrasp generation;
- refined MoveIt pregrasp, grasp, and lift targets;
- all executed geometry and physical grasp checks.

The local boundary check used the same frozen API, 10M map, grasp, and current
BUNKER pose:

| Local-Y angle | Result |
|---:|---|
| `0` | reproducible frozen `acos` domain exception |
| `1e-7 rad` | `ok` |
| `1e-6 rad` | `ok` |
| `1e-5 rad` | `ok` |

For all three nonzero values the top-five IDs and order were exactly
`162, 163, 207, 240, 213`. Across `1e-7` to `1e-5`, the maximum XY change was
about `1.98e-6 m`, every yaw was unchanged, and the maximum score change was
about `6.9e-7`. This confirms the fixed `1e-6 rad` value avoids the numerical
boundary without a material candidate jump. No further epsilon study belongs
to this project stage.

## Target-facing Ground observation

The old fixed joint observation posture is not used in the RM4D-driven path.
For the representative top candidate it places the Brick about 55 degrees off
the D435 optical axis, outside the current horizontal half-FOV.

After navigation and ground stop, the orchestrator instead derives a
target-facing observation/pregrasp pose from the aerial exact grasp TCP:

- use the existing exact side-up grasp generator;
- retain the existing pregrasp safety standoff;
- use the resulting exact pregrasp as the MoveIt target;
- keep the BUNKER at the RM4D candidate pose;
- acquire a fresh Ground D435 observation in `map` from that pose;
- regenerate the exact grasp sequence from the refined target.

This is only the minimum D435 FOV function needed by the natural demo. It does
not add observability scoring, candidate re-ranking, a visibility gate
framework, NBV, or active perception.

## Replay and visualization

One small replay record captures an actual P450 RGB-D observer result, not a
Gazebo model pose. Both replay modes use the same conversion and API code:

- offline mode calls the adapter library under the frozen RM4D environment and
  prints the ranked result;
- online mode sends the same exact grasp through the ROS service and publishes
  the normal topics.

The adapter publishes latched visualization topics in `map`:

- `/rm4d/grasp_tcp`: exact, unregularized grasp TCP;
- `/rm4d/candidates`: ranked `geometry_msgs/PoseArray`;
- `/rm4d/candidate_markers`: arrows, padded BUNKER footprint outlines, and
  rank/score labels.

Top-1 has a distinct color. Remaining markers preserve rank order. RViz gains
only the displays needed for these topics.

## Navigation and execution ownership

The Air-Ground Pick Demo replaces its geometric fixed-standoff goal with the
first RM4D candidate. It sends that candidate unchanged to
`/ground/move_base`, using the existing staged motion behavior where needed but
ending at the exact candidate X/Y/yaw. It then calls the existing ground stop
interface.

RM4D footprint checks are static algorithm checks. Actual reachability remains
owned by:

- `move_base` global/local planner and costmaps for navigation;
- MoveIt planning and collision checking for AUBO motion;
- Gazebo controllers for SIM execution;
- AG95 `grasp_confirmed` and measured TCP lift for grasp success.

V1 does not silently choose another pose when top-1 fails. A navigation,
observation, MoveIt, controller, contact, or lift failure is surfaced as an
execution failure and recorded as an integration/method gap. No Gazebo
teleport, ground-truth substitution, direct plugin command, or illegal object
attachment is allowed.

## Known interface and method gaps

1. RM4D returns manipulation candidates but does not model Ground D435 FOV.
   The target-facing exact pregrasp supplies the minimum required observation
   pose without modifying RM4D ranking.
2. RM4D represents BUNKER placement as SE(2), while the physical SIM
   `ground/base_link` has a nonzero map-frame height. The audited relative
   BUNKER-to-AUBO and flange-to-TCP transforms match, but the frozen RM4D
   vertical IK model is not a substitute for final map-frame MoveIt planning.
3. Static RM4D footprint acceptance does not imply `move_base` path or costmap
   feasibility.
4. RM4D IK validity does not include the complete SIM robot, AG95, D435, scene,
   or the refined sensor pose. MoveIt remains the execution authority.
5. The frozen API needs about one minute for the representative 256-candidate,
   16-start call. It must be loaded once and queried asynchronously from the
   rest of the mission startup, but V1 adds no scheduler or lifecycle system.

## Verification and terminal condition

Implementation proceeds with test-first changes and short commits:

1. adapter request/result conversion and fixed regularization tests;
2. exact external revision and map/API startup checks;
3. offline and online replay of one P450 RGB-D observation;
4. RViz top-K publication;
5. clear-scene top-1 navigation through `/ground/move_base`;
6. target-facing exact pregrasp and Ground D435 refinement;
7. natural Gazebo `P450 -> RM4D -> BUNKER -> D435 -> grasp -> Lift`;
8. focused regression tests plus the existing Air-Ground Pick checks.

`RM4D_SIM_INTEGRATION_READY` is emitted only after the natural E2E reaches
`LIFT`. Work stops at that terminal condition; it does not expand into
uncertainty, MID360, NBV, Task-aware methods, benchmarks, evidence systems, or
new safety frameworks.
