# SIM UAV Map Localization Implementation Plan

> **For agentic workers:** Use subagent-driven-development, with autonomous routine decisions authorized by the user.

**Goal:** Real horizontal-ground returns transformed through public map TF satisfy the existing 0 +/- .02 m ground band, then resume A5.

**Architecture:** A stamped SIM-private P3D body pose and the existing odom/base TF produce one dynamic map/odom localization edge. No estimator or frozen research changes.

**Tech Stack:** Existing Python3.8, NumPy, rospy/tf2, gazebo_ros_p3d, unittest and catkin.

## Task 1 — Same-time localization adapter (one delegated implementer)

Files: create `src/platform/sim_platform_bringup/scripts/sim_uav_localization.py` and `src/platform/sim_platform_bringup/test/test_sim_uav_localization.py` only.

- [ ] RED: test `map_to_odom(T_world_base,T_odom_base,T_map_world)` recomposes exactly: `assert_allclose(result @ T_odom_base, T_map_world @ T_world_base)`. Include a physically landed base at z=.045 with estimated z=-.005, nonidentity roll/pitch/yaw, changing biases, finite/rigid input rejection, and a ground endpoint remaining at map ground height. Run `/usr/bin/python3 -m unittest discover -s src/platform/sim_platform_bringup/test -p test_sim_uav_localization.py -v`; observe missing implementation.
- [ ] GREEN: implement full rigid matrix composition and a lazy-import ROS node. Subscribe `/sim/uav1/base_pose` nav_msgs/Odometry, accept world/base_link, insert into a private tf2 buffer as world/sim_uav1_body. Use existing public tf2 listener to get latest uav1/odom/uav1/base_link TF, look up the private physical pose at exactly that stamp, get map/world, compose, publish map/uav1/odom once per new stamp. Skip transforms older than .5 s, unavailable data and invalid numeric inputs. Keep helper frame private; do not read AGENT, plane fits or change goals. Default node name sim_localization_uav1; 50 Hz attempts.
- [ ] Test the real pure composition and relevant same-time selection helper, ROS import/compile, then commit only owned files. Independent spec then quality review follows.

## Task 2 — SIM composition and regression (root, disjoint files)

Files: P450 normal/MID360 SDF jinja models; two standalone launch files; bringup CMake/package.xml; existing launch contract tests.

- [ ] RED: assert both profiles contain `sim_base_pose` plugin with `libgazebo_ros_p3d.so`, bodyName=base_link, frameName=world, topicName=/sim/uav1/base_pose, updateRate=100 and zero xyzOffset/rpyOffset/gaussianNoise. Assert standalone localization node uses `sim_platform_bringup/sim_uav_localization.py`, no static competing uav1/odom publisher; existing Ground transform unchanged.
- [ ] GREEN: add the small plugin config to both profiles, replace only the old localization node type/package, register Python executable and direct message/tf dependencies in bringup. Update old tests that explicitly expected the erroneous static implementation while retaining public frame/authority assertions.
- [ ] Run focused launch, MID360, D435, Ground, RM4D tests and install only changed catkin packages with existing p450-clean profile. Do not rebuild unrelated native code or modify runtime provenance machinery.

## Task 3 — Live geometry and A5 retry (root)

- [ ] Start one dedicated natural Gazebo runtime with new localization adapter. Observe each TF authority and private stamped pose. At landed and stable hover, transform actual cloud returns with header-stamped public TF; evaluate known horizontal-ground regions, not robot/obstacle endpoints, against z=0 and .02 m tolerance. Record numerical errors and physical body height separately.
- [ ] Start a fresh A5 run with frozen A2 config. Preserve unsuccessful runs; diagnose ordinary interface errors without changing frozen methods. Require actual belief updates, further observation if needed, exact validated candidate, public navigation, D435 refinement, retained grasp and physical lift.
- [ ] Independent review, minimal regressions and documentation; commit platform fix separately from A5 progress. WIP A5 is already allowed to be pushed; do not merge A5 main before genuine E2E success.
