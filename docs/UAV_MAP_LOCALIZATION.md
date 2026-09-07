# SIM public UAV localization correction

This is a SIM platform correctness repair, not an AGENT algorithm change.
The public map remains coincident with world, with horizontal ground at z=0.

## Cause and correction

The previous static map/odom transform used the UAV spawn pose. PX4 local
position has its own datum and time-varying estimator bias, so adding that
spawn pose did not reconstruct physical map/base. The first A5 run consequently
mapped ground above the frozen A2 obstacle threshold. On landing, public
base height exceeded physical height by about 0.1006 m.

The UAV base_link is the center of a 9 cm thick collision box. Its physical
landed height is approximately 0.04484 m, not zero. Neither that structural
height nor the sensor mount translation should be removed.

The existing gazebo_ros_p3d plugin supplies a stamped, zero-noise body pose on
the SIM-private `/sim/uav1/base_pose` topic. Only the SIM localization adapter
uses it. At the newest fresh time shared by physical and estimated pose buffers:

```
T_map_odom(t) = T_map_world(t) T_world_base(t) inverse(T_odom_base(t))
```

Both poses are evaluated at the same time, including interpolation when needed.
The full rigid transform corrects translation and attitude; there is no fitted
ground plane, empirical Z offset, PX4 estimator replacement or goal adjustment.
Delayed streams select a common timestamp rather than chase an unavailable
latest odometry sample. Missing or older-than-0.5-second pairs are not published.

| Public edge | Authority (unchanged) |
|---|---|
| world -> map | /sim_world_to_map |
| map -> uav1/odom | /sim_localization_uav1 |
| uav1/odom -> uav1/base_link | /uav_control_main_1 |
| uav1/base_link -> uav1/lidar_link | /uav_control_main_1 |
| map -> ground/odom | /sim_localization_ground |

The normal and MID360 P450 profiles expose the private body pose. Prometheus
still owns public odom/base; D435/MID360 extrinsics, Ground frames, native flight
control and RM4D interfaces are unchanged. REAL must provide its own localization
adapter; exact simulation localization does not establish real-world robustness.

## Minimal verification

Focused tests cover rigid composition, nonzero landed height, changing estimator
bias, same-time TF interpolation/delivery delay, single authority and the existing
P450, Ground, D435/MID360 and RM4D interface contracts.

`scripts/check_uav_map_ground.py` is a read-only physical geometry check. It
transforms real cloud endpoints using public TF at the cloud header timestamp.
The default known-clear ground patch is x=[6,10], y=[1.5,4] in air_ground_v1.
It tests z=0 directly against 0.02 m tolerance, without fitting/subtracting a bias.
Use `--height-range 0 .1` while landed or `--height-range 1.4 1.6` in stable hover.

First corrected natural run, AGENT `outputs/a5/calibrated-mexIjS`:

| Check | Result |
|---|---|
| Landed, 5 frames / 76 ground returns | max absolute error 0.000000402 m |
| Hover, 10 frames / 389 ground returns | max absolute error 0.0006364 m |
| Public TF authority check | single expected owner for each edge |
| Existing read-only P450/Ground TF and P450 D435 sensor checks | passed |

This establishes the original ground-height repair, separately from A5 task
success. A5 then produced 126 ground/free cell votes from its first real cloud;
remaining occupied returns were actual BUNKER/arm structure (z=0.394..1.028 m)
overlapping candidate footprints. They were not masked or relabeled. That run
correctly stopped without a ground selection and is not a successful E2E result.

After the bounded-delivery-delay review fix, fresh run `clear-parking-iaOxLx`
repeated the ground checks: 5 landed frames / 41 endpoints, max error
0.000000399 m; 10 hover frames / 424 endpoints, max error 0.0004689 m. The
read-only P450/Ground TF and P450 sensor checks again passed. A5 executed two
translated NBV flights and updated belief, but its single-message-per-view
acquisition did not confirm a fully FREE footprint before the three-view guard.
This remaining acquisition issue belongs to A5; no platform or frozen A2
semantics were loosened to continue.

The final localization tests include continuous 20 Hz odometry / 100 Hz physical
poses / 50 Hz update attempts with 60 ms physical-pose delivery delay. The old
latest-odom selection produced zero corrections in 100 attempts. Common-time
selection produced 97, every attempt with available overlapping history. All
18 localization tests passed and independent review closed the timing finding.

## Separate frozen Ground/RM4D boundary discovered during A5

Read-only public TF showed Ground base_link at map z=.36 and AUBO base at .482 m;
their relative installation offset is .122 m. Frozen RM4D's placement generation
instead puts its BUNKER reference at z=0 and AUBO at .122 m. Its .01 m standalone
robot offset does not account for this .36 m difference, and the existing request
adapter performs no height conversion. Both Ground launch geometry and request
behavior predate this UAV localization repair and were not changed here.

The same saved arm-relative target transform consequently reconstructs a TCP
.36 m higher when placed at the SIM mount. This is not a measured MoveIt failure:
the existing execution layer would plan again, and another configuration might
succeed. It does mean nominal RM4D IK/margin validation is not validation of the
same physical target under the current absolute-height contract.

A5 is paused for the user's frozen frame/model decision; no calibration offset,
baseline edit, Ground frame shift or A2 threshold change was introduced. The
full geometry and remaining acquisition issue are documented in AGENT
`docs/A5_GROUND_RM4D_HEIGHT_BOUNDARY.md`. No A5 E2E success is claimed.
