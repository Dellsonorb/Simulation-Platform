# Ground XY latch namespace correction (development)

This source-only correction moves the existing intended `true` value from
`/ground/move_base/DWAPlannerROS/latch_xy_goal_tolerance` to
`/ground/move_base/latch_xy_goal_tolerance`. XY/yaw tolerances (0.06 m/0.08 rad),
velocities, planner frequency, collision checks and controller code are unchanged.

ROS navigation 1.17.3's [DWA constructor and setPlan](https://github.com/ros-planning/navigation/blob/1.17.3/dwa_local_planner/src/dwa_planner_ros.cpp#L87)
default-construct the stop/rotate controller and reset its latch on a new plan.
The [stop/rotate constructor](https://github.com/ros-planning/navigation/blob/1.17.3/base_local_planner/src/latched_stop_rotate_controller.cpp#L22)
reads the node-private namespace, defaulting to false. The former nested YAML
entry was therefore unused by that controller. The installed navigation package
in the development SIM is 1.17.3.

## Runtime evidence motivating the fix

AGENT development `launch-07-easy-ours` used public map-frame goal
(2.410300620, -0.633293445, 2.094395102). During its unchanged 120 s navigation
timeout, 6000 public TF samples showed minimum XY error 0.058028 m, but no
sample simultaneously met XY and yaw tolerances. Final errors were 0.061414 m
and 0.259105 rad. Goal/robot cells in the final costmap were clear. The master
log contained only the incorrect DWA-nested latch parameter. This establishes
a parameter-binding bug and a near-goal oscillation, not exclusive causality.

## Validation and remaining limitation

The focused configuration regression fails on the old YAML and passes after
the move. It also verifies unchanged tolerances and node-private launch loading.
Package/config and platform source regressions are recorded in the AGENT batch
report. No new navigation code or AGENT method change is introduced here.

**Not online validated or deployed to the existing p450-clean installation.**
The bounded AGENT batch's last reserve repeated Hard/Ours after a pre-method
PX4 startup INVALID_TRIAL; the proposed focused navigation launch was cancelled to keep
the 12-start cap. All development pairs used the old installed SIM 5e25039.
Do not claim this patch cures the timeout: 2 Hz global replanning may reset the
latch even after the namespace correction. A focused public-goal regression
after normal package installation remains necessary, without relaxing limits
or erasing the original valid navigation failure.
