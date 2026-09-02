# Simulation Platform V1.0

This repository provides the reusable P450 + BUNKER + AUBO i5 + AG95 ROS 1
runtime. It targets Ubuntu 20.04, ROS Noetic, Gazebo 11, PX4, MAVROS,
Prometheus, MoveIt, and `move_base`. `P450-PAPER` is read-only source material;
paper benchmarks and research algorithms are outside this repository's runtime.

The architectural boundary is:

```text
AGENT or task -> common runtime -> official native robot stack
```

Gazebo supplies the current SIM backend. Upper task and future AGENT code must
not call Gazebo APIs or use its `world` frame.

## SIM/REAL robot-facing contract

P450 exposes:

- `/uav1/runtime/flight` (`FlightCommandAction`) with `TAKEOFF`, `FLY_TO`,
  `HOVER`, and `LAND`;
- native Prometheus `/uav1/prometheus/state`, `control_state`, and `odom`;
- standard RGB-D topics below `/uav1/camera/` and MID360
  `/uav1/livox/lidar` (`sensor_msgs/PointCloud2`).

`FlightCommand` is only a thin translator to Prometheus
`UAVCommand/UAVSetup/UAVState/UAVControlState`. Prometheus remains the flight
state-machine and control authority.

BUNKER exposes:

- `/ground/move_base` (`MoveBaseAction`) and `/ground/runtime/stop`;
- the hardware boundary `/ground/cmd_vel`, `/ground/odom`,
  `/ground/bunker_status` (`bunker_msgs/BunkerStatus`), and TF;
- `/ground/scan` and `/ground/imu/data` using standard ROS sensor messages.

`move_base` is shared navigation above the hardware boundary. SIM implements
only ROS interface fidelity; it does not emulate `ugv_sdk` or CAN.

Manipulation keeps the existing MoveIt and trajectory-controller interfaces.
The backend-neutral retained-object result is
`/ground/gripper/grasp_confirmed`. SIM derives it from strict bilateral Gazebo
contacts plus gripper position; a real AG95 adapter may use position, current,
status, or tactile feedback.

## Localization Adapter Contract

Both robots must enter one shared `map`, with exactly one TF authority for each
edge:

```text
map -> uav1/odom -> uav1/base_link
map -> ground/odom -> ground/base_link
```

Gazebo `world` is private to the SIM localization adapter. A REAL deployment
may provide the two `map -> odom` edges through RTK, VIO, SLAM, EKF, or a chosen
fusion stack without changing upper code.

## REAL replacement boundary

For P450, replace PX4 SITL, the Gazebo vehicle/sensors, and SIM localization
with the official `p450_experiment + Prometheus + MAVROS + PX4` deployment,
real D435/MID360 drivers, and the selected localization adapter. Keep the
flight facade and all of its clients unchanged.

For BUNKER, replace the Gazebo base/sensors and SIM localization with official
`bunker_base -> ugv_sdk -> CAN`, real LiDAR/IMU drivers, and the selected
localization adapter. Keep `move_base`, its goals, the stop service, MoveIt,
manipulation controllers, and upper task code unchanged.

## Main launches

- `sim_platform_bringup/air_ground_standalone.launch`: joint SIM backend plus
  common flight and navigation layers.
- `air_ground_pick_demo/air_ground_pick_demo.launch`: the natural one-flight,
  conditional-ground-move, near-field observation, grasp, and lift task.

The existing scripts under `scripts/smoke_*` provide bounded platform and
natural E2E checks. They are ordinary runtime tests, not benchmark or paper
infrastructure.
