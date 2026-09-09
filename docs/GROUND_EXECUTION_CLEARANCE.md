# Bounded Ground execution clearance (development)

Implementation `1f47fe4`, based on preserved manipulation version `625b84c`.
`~execution_clearance=true` additionally requires `~full_robot_manipulation`.
Historical mode remains available; this is not a calibrated safety guarantee.

## Geometry and selection

The authoritative BUNKER URDF collision box is duplicated in the planning
scene with 12 mm added to each half-extent. The allowance rounds the largest
corresponding collision-point tracking displacement, 11.490625 mm in 26,683
prior recorded arm samples, upward to 1 mm. It is fixed before new online
outcomes. Only the all-fixed chain from the model root is exempt from this
copy; arm-mounted camera and finger assemblies remain checked. The original
robot, self collisions, floor and target geometry are unchanged. Other pairs
do not acquire a claimed positive margin.

The AGENT integration screens at most four already-confirmed exact per-cell
winners in existing relevance/stable order. SIM previews each using the
runtime-perceived target, known brick dimensions and public base height. Two
brick-symmetry grasp yaws and three deterministic seeds form a bounded search;
no non-winner, unconfirmed pose or simulator target pose is introduced. Aerial
preview does not guarantee camera observation, navigation or actual arrival.
Both Generic and Ours use the same common execution layer.

At the actual station, accepted D435 refinement triggers a new whole-chain
check before pregrasp. Check approach, descent, physical closure and prospective
attached lift with the full BUNKER/AUBO/AG95/D435 model. Only existing intended
finger/target contacts are allowed. The loaded articulation envelope extends
from geometry-derived contact to 0.522 rad (previous observed maximum
0.521259581 rounded upward by 1 mrad); it does not change the 0.70 force command.
Exceeding the checked range stops execution rather than silently extrapolating.

Prospective payload geometry uses a distinct planning-only API. Every search
exit restores the original world, attachments and collision matrix. Native
MoveIt testing found that setting a full scene does not remove unspecified
attachments; introduced hypotheses therefore receive explicit REMOVE before
snapshot restoration. A real attachment still requires fresh physical grasp
confirmation and measured TCP geometry. No physical weld is created.

## Actual commands and runtime correctness

The opt-in path dispatches through the existing serial FollowJointTrajectory
action client with unchanged controller tolerances. Before each command, require
a fresh stationary controller DESIRED hold and no pending/active action. Sample
the retimed linear/cubic/quintic curve, including the real held-desired start
bridge, at at most 1 ms spacing. This is sampled checking, not continuous
collision certification or an exact global controller-time lattice. Preserve
complete measured state and attached payload. Loaded checks cover the same
curve and start bridges across the checked articulation range. A second fresh
state check precedes dispatch, and actual endpoint validity follows completion.
An aborted/timed-out arm action is a failure, not a reason for execution retry.
Incomplete or interface-failed checks cannot count as candidate infeasibility
or success. Geometric/planning rejection alone permits the next bounded branch.

Accepted target transforms for both symmetry poses are resolved before lengthy
planning/preshape work; selected planning-frame poses are reused without asking
for an expired historical TF. Separately, `157f296` fixes initial target-scene
startup by waiting for fresh public TF within the existing 500 ms wait and age
limits. It neither extends the buffer age threshold nor substitutes GT.

The SIM integrated-pose velocity backend remains explicitly configured and
retains native diagnostics. No filter, collision exemption, modified contact
height, physical grip condition or retention threshold is introduced here.

## Verification and online scope

Root native tests: 94 pass, including C++ planning-scene round trips. Independent
specification and code reviews found no remaining actionable issue before
online activation. The `p450-clean` package build/install passes. These checks
do not establish robot success; online results will be recorded in AGENT's
`docs/GROUND_CLEARANCE_BATCH_RESULTS.md` with a maximum of six starts including
startup failures. Historical failures remain unchanged. No formal matrix.

Local start2 rejects source624 before pregrasp after successful fresh camera
observation. Local start3 at source664 passes whole-chain preflight, actual
pregrasp/D_exec, descent and real grip/attachment; it then times out during the
loaded lift check before any lift command. That failure remains preserved.
The pregrasp's 15,494-state check was already near 60 s. A transport-only
native TCPROS contrast (not MoveIt timing) measures 300 connections versus one,
0.503 versus 0.119 ms mean per call, with identical responses. Reuse the
serialized read-only validity connection in clearance mode only. Connection
failures still propagate; no retry, dropped samples or increased guard budget.
Actual recovery remains for the separately declared engineering regression.
