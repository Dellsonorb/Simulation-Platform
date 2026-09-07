# SIM UAV public-map localization correction

The user authorizes this platform correctness repair and ordinary implementation
decisions autonomously. AGENT A1-A4 methods, thresholds and ground_z stay frozen.
Branch: feature/fix-sim-uav-map-localization, based on SIM 30b645b.

## Root cause and preserved semantics

world -> map is identity, authority /sim_world_to_map. map -> uav1/odom currently
uses a spawn-pose static transform, authority /sim_localization_uav1. The
uav1/odom -> uav1/base_link authority /uav_control_main_1 broadcasts PX4 local
position and IMU orientation. PX4's estimator datum/drift is not the spawn pose.
The resulting apparent ground elevation crossed A2's 5 cm obstacle threshold.
base_link has a 9 cm thick centered collision box: about 4.5 cm physical landed
height is correct and must not be removed. Camera and MID360 composite extrinsics
already include their sensor origins and are unchanged.

## Selected repair

Keep the frame tree, edge authorities, map/world identity, PX4 native control and
all Ground edges. Replace only the SIM map -> uav1/odom static publisher with a
dynamic localization adapter of the same node name. A SIM-private stamped body
pose comes from the already installed gazebo_ros_p3d plugin, bodyName base_link,
frameName world, zero noise/offset, 100 Hz, topic /sim/uav1/base_pose. It does not
publish public TF or feed the AGENT directly. Normal and MID360 model profiles
both expose this localization input.

At the newest fresh timestamp t common to both pose histories (the earlier of
their latest stamps, with both poses queried at exactly t), publish:

    T_map_odom(t) = T_map_world * T_world_base(t) * inverse(T_odom_base(t))

Use full SE(3), not scalar altitude subtraction. Cache the private pose in a local
tf2 buffer, never broadcast its helper frame. Do not use clamped display odometry
(the existing Odometry topic clamps z<=0 to .01 while TF does not). Publish once
per matched stamp; skip unavailable/stale transforms without inventing a pose.
Choosing the common time avoids starving localization when one continuously
advancing stream is delivered later than the other.
The existing node remains the only map -> odom authority. No new sensor estimator,
SLAM package, localization benchmark or safety framework is introduced.

Alternatives rejected: changing spawn Z or a one-time Z offset cannot remove
time-varying translation/attitude error; changing PX4/GPS control localization
would affect much more of the verified flight stack. The selected correction
implements the existing SIM Localization Adapter Contract with exact simulator
localization. REAL must supply its own RTK/VIO/SLAM/EKF adapter. This is not a
claim of real localization robustness or a change of public map semantics.

## Verification

Unit tests reconstruct physical base/sensor/ground under drift, nonzero landed
base height, rotations and translated world/map; verify same-time pairing and
single authority in launch. Run focused P450, MID360/D435, Ground and RM4D public
contract regressions. Live checks consume real stamped ground returns at landed
and stable hover poses and require errors within the frozen .02 m tolerance.
Only then retry A5 with a new initial cloud. Dynamic correction is not an obstacle
map and does not relabel BUNKER/body returns or guarantee a ground candidate.
