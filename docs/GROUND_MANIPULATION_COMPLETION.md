# Ground manipulation completion: development correction

This branch extends `feature/fix-ground-execution-reliability @ 7d2abf6`.
Historical failures and the original native-feedback configuration remain
available. These are simulator/execution changes, not changes to the RM4D
baseline, environment evidence, or research scoring. No physical attachment
or simulator target pose is used to drive the algorithm.

## QuickStep feedback semantics

The installed Gazebo11.15.1 ODE QuickStep solver updates body poses using
constraint ERP correction, then removes the ERP contribution from its stored
body velocity (`quickstep_update_bodies.cpp`, `erp_removal=1`). Thus stored
hinge rate need not equal the derivative of the integrated joint configuration.

A failed loaded wrist hold sampled at every1ms physics step measured only
−0.00001788rad net rotation, but the integral of the always-negative native
hinge rate was−0.336535rad. Independent parent/child quaternion rotation agrees
with the small angle change. Native hinge rate agrees with the projection of
stored relative angular velocities; this is not a ROS subscriber readout bug.
Immediate base velocity setters changed neither angle nor rate. Partial real
gripper opening substantially reduced the native wrist bias while the object
remained held, implicating the gripping constraint regime rather than proving
an unloaded-weight effect. The direct ODE `world` solver contrast caused LCP
numerical errors and was rejected; default physics remains QuickStep.

`IntegratedVelocityRobotHWSim` subclasses the installed DefaultRobotHWSim.
By default it preserves native feedback. Setting
`P450_GROUND_INTEGRATED_VELOCITY=1` before model load explicitly selects
**integrated-position interval-average joint velocity**: the change in stock
unwrapped position divided by the consecutive hardware-read interval. All
controlled joints use the same rule, without filtering, clipping, deadbands,
quiet-sample selection or modified stop thresholds. First/reset/invalid samples
retain native feedback and prime/re-prime the previous-position state. Native
nonfinite rates are not concealed. Registered velocity storage is updated in
place; `writeSim`, position PID, joint limits, gains and physical contacts are
unchanged. This is a feedback-interface convention, not a claim that the
post-ERP native velocity or ODE solver has been repaired.

Before activation, nonzero unloaded wrist motions of2.45–3.20rad and loaded
holds independently compared joint position increments with relative-link
quaternion motion. After activation in development launch05, both public joint
state and controller actual velocity for all six arm joints matched the
same-stamp preceding1ms position increments exactly. Independent wrist
quaternion disagreement was below0.003887rad/s maximum in observation and
8e-8rad/s in loaded lift/retention; no timestamp shifting or interpolation was
used. Other arm joints have direct position references, not simultaneous
per-joint quaternion columns. The seventh/gripper joint is covered by shared
numeric tests and public motion records, but has no independent native1kHz
column in this trace.

Physical outcome, separately from action success: TCP149.75mm and target149.41mm
rise, maximum target-in-hand motion1.466mm during lift and0.145mm during the
existing0.797s retention interval. This single contrast is not a reliability
estimate or a prolonged-hold test. Raw native velocities remain available in
the optional diagnostic CSV and still show the discrepancy.

Implementation sources: [Gazebo11.15.1 QuickStep body update](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/deps/opende/src/quickstep_update_bodies.cpp#L245),
[DefaultRobotHWSim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_ros_control/src/default_robot_hw_sim.cpp#L201).

## Gripper release trajectory start

After a contact-stalled close, the controller's desired closure can remain
0.700rad although actual position is approximately0.517rad. A new opening
trajectory that implicitly starts at the stale desired position immediately
violates the unchanged0.08rad path tolerance. Opening now supplies a fresh
measured position and velocity waypoint at time zero, with the existing future
trajectory stamp. Endpoint, duration, closure behavior and tolerances remain
unchanged. Invalid or stale measurements do not result in a sent goal.

The focused loaded release contrast removes the immediate discontinuity, but
later actual opening lag still violates the same path constraint under native
QuickStep feedback. Do not interpret this fix as a demonstrated full release.

## Full-robot perceived-object planning (development opt-in)

`~full_robot_manipulation` defaults to false. Its opt-in path uses the existing
whole BUNKER/AUBO/AG95 MoveIt robot, accepted runtime aerial/refined target box,
and known target dimensions. Partial D435 surface cues are not cuboid geometry.
Only the intended finger contact assembly may touch the target during closure;
all chassis, arm, wrist and knuckle checks remain active. A planning attachment
is made only after real grasp confirmation, using fresh measured TCP geometry;
it never creates a physical constraint in Gazebo. Attached geometry is retained
in subsequent planning/start-state requests.

Target-sized preshape is derived from the existing conservative aperture rule,
required object width plus opening margin, and existing tracking tolerance.
It is a proposed actuator command, not a collision certificate: the actual
measured aperture and whole-robot collision state must pass. In Moderate,
fully open fingers/knuckles overlap the chassis at the target TCP independent
of arm IK. Nominal narrower preshape removes that hand-only overlap, but its
tracking interval includes collisions; whole-model measured-state/planning
checks cannot be replaced by the nominal calculation.

The native MoveIt regression must cover world-object round-trip serialization,
object-level plus shape-local poses, attachment geometry, and preservation of
payloads and unrelated ACM entries. Mock ROS message tests alone missed the
initial attachment-composition and redundant-REMOVE defects; these were found
offline before activating the new planning path.

## Contact-configuration grasp height

Checkpoint86abd7a preserves the initial full-robot path with its old
fully-open pad-height assumption (used in AGENT development starts06/07).
Start07 completes aerial confirmation, real navigation, refinement, pregrasp
and descent, then its hypothetical closure sweep rejects an inner-knuckle/
perceived-target intersection before any physical close command.

Rendered URDF and pad collision-mesh FK independently show that the existing
`finger_pad_lower_edge_offset=0.0156m` describes the fully-open pad edge above
TCP. Parallel finger closure changes that offset. For link length L=.055m and
alpha=44.691deg, the actual aperture is
`w(q)=.0952+2L[cos(alpha+q)-cos(alpha)]`. The physical53mm target width gives
q_contact=.4573744071rad and downward pad displacement
`delta=L[sin(alpha+q_contact)-sin(alpha)]=.0132905621m`.

The opt-in Ground generator uses `pad_edge_contact=.0156-delta`, retaining the
existing20mm overlap, relative pregrasp/lift heights, yaw and XY. Thus refined
Ground TCP rises13.290562mm; this is not an outcome-fitted Z offset. The
nominal pad mesh then has20.040562mm overlap (the40.56µm difference is the
existing open-edge calibration rounding). Physical target width, not preshape
width plus margin, determines q_contact. The same helper now drives closure
geometry, and the A5 grasp-seeded approach uses the same Ground generator.
Default/legacy paths and initial aerial/RM4D queries remain unchanged; refined
Ground geometry is collision-aware revalidated before execution.

An offline41-sample comparison using start07's accepted target and public
TF/JointState reproduces seven terminal forbidden intersections with old
measured geometry and none with the correction. The old nominal geometry
alone is collision-free: excessive insertion plus actual tracking error
causes the veto. The recorded lateral error still produces one-sided pad
contact first; this correction is not a bilateral-contact or payload-lift
certificate. No knuckle collision allowance, observation/force threshold,
tracking limit, target dimension, or real grasp criterion is relaxed.

## Final bounded online disposition

AGENT's batch ends at8/8 Gazebo starts, with no formal run or retry after the
cap. On the other archived Moderate station (candidate000009), code c12d8ba
passes the complete arrival-conditioned Ground chain with this correction:
real D435 refine, first-branch pregrasp and descend, sampled whole-robot closure,
fresh confirmed physical grip, modeled payload/current-state validity and
collision-aware lift. TCP rises149.537mm and physical target148.902mm; existing
0.581s retention stays confirmed, relative target-hand movement≤0.04043mm.
The gripper action contact-stalls at the original path limit and is accepted
by the existing real-confirmation gate, not ordinary action success.

Public six-arm feedback again exactly matches same-stamp preceding1ms pose
increments, independently checked wrist quaternion-rate error≤1.02e-7rad/s
during lift. Raw native discrepancy remains. Together with Easy05 this supports
the feedback convention in two specific executions, not general reliability.

The nominal clearance at the separate Moderate source624 remains too small
for the recorded actual TCP tracking error: measured finger/chassis collision
properly blocks descent. No tiny-collision exemption or tracking relaxation
was added. The sole aerial E2E07 is still a failure, before the contact-height
correction; the corrected full aerial chain has **not** been re-run. Candidate
clearance/actual execution robustness remains development work.

Final relevant checks:109 SIM tests and133 native AGENT tests pass without
skips;645 core tests pass with22 environment skips (overlapping suites).
Native Pluginlib construction, actual mesh calibration, and native MoveIt
scene application are included. Builds/install pass. Keep the correction
feature branch and PR Draft; historical defaults/results remain available.

Final bounded online results and limits are recorded in the AGENT repository's
`docs/GROUND_MANIPULATION_BATCH_RESULTS.md`; raw/derived development data live
under `outputs/development/ground-manipulation-batch/`. No formal run is included.
