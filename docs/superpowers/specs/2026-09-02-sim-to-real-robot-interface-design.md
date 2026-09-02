# Sim-to-Real Robot Interface Design

**Date:** 2026-09-02  
**Status:** Approved for implementation  
**Scope:** `SIM/` only

## Purpose

Provide the minimum robot-facing interface needed to run the existing P450 +
BUNKER + AUBO i5 + AG95 platform in Gazebo now and migrate the same upper-level
task and AGENT code to the future real P450 and BUNKER stacks.

The design does not replace the official robot stacks. Its target flow is:

```text
AGENT / task code -> common runtime -> official native robot stack
```

The implementation must not add benchmark, paper, evidence-chain, formal,
Pilot, adversarial-safety, or complex lifecycle infrastructure.

## Design principles

1. Preserve the official P450 control chain:
   `p450_experiment -> Prometheus -> MAVROS -> PX4`.
2. Preserve the official BUNKER driver chain:
   `bunker_ros -> bunker_base -> ugv_sdk -> CAN`.
3. Keep the P450 facade thin. Prometheus remains the only flight-state and
   control authority.
4. Keep `move_base` above the BUNKER hardware boundary and use the same
   navigation layer in SIM and REAL.
5. Make `map` the only shared global frame visible to upper code. Gazebo
   `world` is SIM-backend-only.
6. Use driver-native or standard ROS messages and launch remapping. Do not
   relay high-bandwidth RGB-D or point-cloud streams.
7. Simulate interface fidelity, not unnecessary hardware internals. The SIM
   BUNKER adapter emulates ROS topics, status, odometry, and TF; it does not
   emulate `ugv_sdk`, CAN frames, or motor electronics.

## Runtime architecture

```text
Air-Ground Pick Demo / future AGENT
  |-- P450 FlightCommand facade
  |-- BUNKER move_base + stop helper
  |-- standard pose / TF / sensor topics
  `-- existing MoveIt and trajectory-controller interfaces
                         |
                    common runtime
                         |
             +-----------+-----------+
             |                       |
        SIM backend              REAL backend
   PX4 SITL + Prometheus    p450_experiment + Prometheus
   Gazebo P450 plugins      MAVROS + physical PX4
   Gazebo BUNKER adapter    bunker_base + ugv_sdk + CAN
   Gazebo sensors           official sensor drivers
   SIM localization         real localization adapters
```

Common-runtime nodes and task code must not import `gazebo_msgs`, call Gazebo
services, subscribe to Gazebo ground truth, or use `world` as a target frame.

## P450 interface

### Native backend contract

Prometheus remains authoritative for:

- `/uav1/prometheus/command` (`prometheus_msgs/UAVCommand`)
- `/uav1/prometheus/setup` (`prometheus_msgs/UAVSetup`)
- `/uav1/prometheus/state` (`prometheus_msgs/UAVState`)
- `/uav1/prometheus/control_state`
  (`prometheus_msgs/UAVControlState`)
- `/uav1/prometheus/odom` (`nav_msgs/Odometry`)
- MAVROS and PX4 setpoint, arming, mode, estimator, and failsafe behavior

The `uav1` namespace is retained because it is the official Prometheus naming
convention used by the current stack. In generic diagrams, `uav/...` means the
configured UAV namespace; V1 uses `uav1/...`.

References:

- [Prometheus UAV controller](https://github.com/amov-lab/Prometheus/blob/main/Modules/uav_control/src/uav_controller.cpp)
- [Prometheus UAV estimator](https://github.com/amov-lab/Prometheus/blob/main/Modules/uav_control/src/uav_estimator.cpp)
- [PX4 ROS 1 and MAVROS](https://docs.px4.io/v1.14/en/ros/ros1)

### Public flight facade

The common runtime exposes one action server:

```text
/uav1/runtime/flight    robot_runtime_interfaces/FlightCommandAction
```

Action goal:

```text
uint8 TAKEOFF=1
uint8 FLY_TO=2
uint8 HOVER=3
uint8 LAND=4

uint8 command
geometry_msgs/PoseStamped target  # used only by FLY_TO
```

Result:

```text
bool success
string message
```

Feedback:

```text
geometry_msgs/PoseStamped current_pose
float32 position_error
string native_mode
```

The client API exposes `takeoff()`, `fly_to(pose)`, `hover()`, and `land()`.
`FLY_TO` accepts a TF-resolvable pose; upper tasks use `map`.

The facade performs translation and completion observation only:

| Public command | Prometheus translation |
| --- | --- |
| `TAKEOFF` | Use native `UAVSetup` to arm, enter `COMMAND_CONTROL`, and select `OFFBOARD`, then issue `UAVCommand::Init_Pos_Hover` |
| `FLY_TO` | Transform the target from `map` to `uav1/odom`, then issue `UAVCommand::Move` with `XYZ_POS` |
| `HOVER` | Issue `UAVCommand::Current_Pos_Hover` |
| `LAND` | Issue `UAVCommand::Land` |

The facade must not implement a position controller, publish MAVROS setpoints,
derive a second flight-state machine, or reinterpret native failsafe behavior.
It uses `UAVState` and `UAVControlState` as the single source of truth. Native
Prometheus `control/Takeoff_height` remains the takeoff-height authority.

Canceling a `FLY_TO` action translates to `Current_Pos_Hover`. A stale or
disconnected backend, invalid odometry, or native failsafe aborts the action
without overriding Prometheus recovery behavior. Operation deadlines remain
caller/task concerns; no SIM-only action timeout is part of the contract.

### Public state and pose

No duplicate universal state message is introduced. These native topics are
already identical between the intended SIM and REAL P450 stacks:

| Topic / TF | Type / authority |
| --- | --- |
| `/uav1/prometheus/state` | `prometheus_msgs/UAVState` |
| `/uav1/prometheus/control_state` | `prometheus_msgs/UAVControlState` |
| `/uav1/prometheus/odom` | `nav_msgs/Odometry` |
| `uav1/odom -> uav1/base_link` | Prometheus estimator |

## BUNKER interface

### Common navigation layer

The common navigation layer is identical in SIM and REAL:

| Interface | Type / meaning |
| --- | --- |
| `/ground/move_base` | `move_base_msgs/MoveBaseAction` |
| `/ground/runtime/stop` | `std_srvs/Trigger`; cancel the active goal and command zero velocity |
| `/ground/nav_cmd_vel` | raw `move_base` output |
| `/ground/cmd_vel` | bounded command delivered to the hardware boundary |

Data flow:

```text
/ground/move_base
      -> /ground/nav_cmd_vel
      -> existing velocity guard
      -> /ground/cmd_vel
      -> SIM Gazebo adapter OR REAL bunker_base
```

The velocity guard remains a simple correctness boundary for finite values,
platform speed limits, and stale-command stop. It is not expanded into a new
safety framework.

### Hardware boundary

The common BUNKER hardware boundary is:

| Interface | Type |
| --- | --- |
| `/ground/cmd_vel` | `geometry_msgs/Twist` |
| `/ground/odom` | `nav_msgs/Odometry` |
| `/ground/bunker_status` | `bunker_msgs/BunkerStatus` |
| `ground/odom -> ground/base_link` | one odometry authority |

The SIM adapter publishes this boundary directly. It does not simulate
`ugv_sdk` or CAN. The REAL launch starts official `bunker_base` and remaps its
absolute topic names into the `ground` namespace while setting:

```text
odom_topic_name=/ground/odom
odom_frame=ground/odom
base_frame=ground/base_link
```

References:

- [AgileX bunker_ros](https://github.com/agilexrobotics/bunker_ros)
- [bunker_base node](https://github.com/agilexrobotics/bunker_ros/blob/master/bunker_base/src/bunker_base_node.cpp)
- [Bunker ROS messenger](https://github.com/agilexrobotics/bunker_ros/blob/master/bunker_base/src/bunker_messenger.cpp)
- [BunkerStatus message](https://github.com/agilexrobotics/bunker_ros/blob/master/bunker_msgs/msg/BunkerStatus.msg)

The SIM `BunkerStatus` contains simulated observable chassis state. Nominal
values may represent unavailable electrical details, but no CAN protocol or
motor-electronics simulation is required.

The SIM odometry must use the same local-odometry semantics as `bunker_base`:
pose starts at the local origin and integrates base motion. Gazebo absolute
`WorldPose()` must not be published as `/ground/odom`.

## Localization Adapter Contract

Localization is an explicit replaceable boundary. It is not tied to any
specific RTK, VIO, SLAM, MID360, or EKF implementation.

Required outputs are:

```text
map -> uav1/odom
map -> ground/odom
```

Contract rules:

1. Both robots must be connected to the same `map` frame.
2. Each `map -> odom` edge has exactly one authority.
3. Each robot's native estimator/driver owns only its `odom -> base_link` edge.
4. Upper code consumes `map` and never consumes Gazebo `world`.
5. Localization adapters may change between SIM and REAL without changing
   public topics, task code, navigation, or manipulation.

SIM TF:

```text
world                         # SIM backend only
`-- map                       # identity SIM anchor
    |-- uav1/odom             # SIM localization adapter
    |   `-- uav1/base_link    # Prometheus
    `-- ground/odom           # SIM localization adapter
        `-- ground/base_link  # BUNKER SIM odometry
```

REAL TF:

```text
map
|-- uav1/odom                 # selected real localization adapter
|   `-- uav1/base_link        # Prometheus
`-- ground/odom               # selected real localization adapter
    `-- ground/base_link      # bunker_base
```

Spawn offsets belong to the SIM localization adapters, not to the local odom
messages. Gazebo `world` is permitted only in SIM launch, spawn, model, and
localization-adapter code.

## Sensor contract

SIM plugins publish the canonical topics directly. REAL drivers use their
normal configuration and launch remapping. High-bandwidth relay nodes are not
part of the design.

| Robot / sensor | Topic | Type | Frame |
| --- | --- | --- | --- |
| P450 D435 color | `/uav1/camera/color/image_raw` | `sensor_msgs/Image` | `uav1/camera_color_optical_frame` |
| P450 D435 color info | `/uav1/camera/color/camera_info` | `sensor_msgs/CameraInfo` | same optical frame |
| P450 D435 depth | `/uav1/camera/depth/image_raw` | `sensor_msgs/Image` | `uav1/camera_depth_optical_frame` |
| P450 D435 depth info | `/uav1/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | same optical frame |
| P450 D435 cloud | `/uav1/camera/depth/color/points` | `sensor_msgs/PointCloud2` | D435 optical frame |
| P450 MID360 | `/uav1/livox/lidar` | `sensor_msgs/PointCloud2` | `uav1/lidar_link` |
| BUNKER 2D LiDAR | `/ground/scan` | `sensor_msgs/LaserScan` | `ground/lidar_2d_link` |
| BUNKER IMU | `/ground/imu/data` | `sensor_msgs/Imu` | `ground/imu_link` |

The active MID360 Gazebo profile changes from
`prometheus_msgs/LivoxCustomMsg` to the plugin's existing
`sensor_msgs/PointCloud2` mode. The real Livox driver is configured or remapped
to the same topic and frame.

The BUNKER base driver remains independent of LiDAR and IMU drivers, matching
the official AgileX package boundary.

## Manipulation and grasp state

Existing manipulation interfaces remain unchanged:

```text
/ground/arm_controller/follow_joint_trajectory
/ground/gripper_controller/follow_joint_trajectory
/ground/joint_states
MoveIt group: manipulator
TCP: ground/gripper_tcp_link
```

The backend-neutral grasp confirmation interface is:

```text
/ground/gripper/grasp_confirmed    std_msgs/Bool
```

It means that the active backend has enough evidence to accept that the target
is retained for lift. It does not promise bilateral tactile sensing.

- SIM derives `grasp_confirmed` from strict bilateral Gazebo contacts plus the
  existing gripper-position checks.
- REAL may derive it from AG95 position, current, status, tactile data, or a
  minimal combination available on the installed gripper.
- Upper code must not import Gazebo contact messages or assume two independent
  real contact channels.

No teleport, ground-truth substitution, or illegal attachment is permitted.

## Simulator-specific coupling boundary

The following are allowed only below the SIM backend boundary:

- PX4 SITL and Gazebo motor/controller plugins
- Gazebo P450 and BUNKER model spawning
- the BUNKER Gazebo motion plugin
- Gazebo RGB-D, LiDAR, MID360, and IMU plugins
- `world -> map` and SIM spawn alignment
- target spawning and Gazebo contact-to-`grasp_confirmed` translation
- Gazebo service and startup timeouts
- SIM-only tests that inspect Gazebo model state

The following must be removed from common runtime and the Air-Ground Pick Demo:

- direct Prometheus command/setup publication by the Demo
- direct `/ground/cmd_vel` publication by the Demo
- runtime imports of `gazebo_msgs`
- `world` target or perception frames
- `/prometheus/ground_truth` TF bridges
- active P450 ROS ground-truth publishers
- Gazebo absolute pose labeled as BUNKER odometry
- simulator-only task timeouts or plugin-control calls

Legacy packages may retain clearly inactive SIM-only utilities, but common
bringup, public launch files, and the Demo must not depend on them.

## Replaceable REAL adapters

### P450

Replace only:

- PX4 SITL and Gazebo model launch with official `p450_experiment`
- simulated transport with physical MAVROS/PX4 transport
- SIM localization adapter with the selected RTK/VIO/SLAM/EKF adapter
- Gazebo D435/MID360 plugins with official device drivers

Keep unchanged:

- `FlightCommand` facade and client API
- Prometheus native command and state contract
- upper task sequence, perception, and target handoff

### BUNKER

Replace only:

- Gazebo BUNKER adapter with official `bunker_base`
- SIM localization adapter with the selected real localization adapter
- Gazebo LiDAR/IMU plugins with real sensor drivers

Keep unchanged:

- `/ground/move_base`
- costmaps and planners
- velocity guard and stop helper
- `/ground/cmd_vel`, odom, status, and TF names
- upper task sequence and manipulation

### AG95 grasp confirmation

Replace the Gazebo contact adapter with the minimal real AG95 feedback adapter.
The public `grasp_confirmed` topic and upper grasp/lift logic remain unchanged.

## Error handling

- P450 facade goals are rejected when native connection or odometry is absent.
- Native Prometheus failsafe remains authoritative; the facade only reports it.
- BUNKER navigation failures use normal `move_base` action results.
- `stop` cancels navigation and commands zero velocity through the same common
  command path.
- Sensor and TF freshness checks remain in task/perception code because they
  affect robot correctness in both SIM and REAL.
- MoveIt execution, joint limits, controller results, collision checks, and
  physical grasp confirmation remain required.

No additional safety state machine or lifecycle barrier is introduced.

## Verification

Implementation is complete only when all of the following hold:

1. Contract/unit tests prove every facade command maps to the expected native
   Prometheus command and completion predicate.
2. The Demo no longer publishes Prometheus native commands or BUNKER velocity.
3. The Demo runtime has no `gazebo_msgs` dependency.
4. Common configs and upper code contain no `world` frame dependency.
5. BUNKER SIM publishes compatible `/ground/cmd_vel`, `/ground/odom`,
   `/ground/bunker_status`, and TF interfaces.
6. BUNKER odom is local rather than Gazebo absolute world pose.
7. P450 RGB-D and MID360 plus BUNKER LiDAR and IMU publish the specified message
   types, frame IDs, and fresh timestamps.
8. Both robots have one connected TF tree through the Localization Adapter
   Contract, with one authority per edge.
9. Existing MoveIt, controller, collision, joint-limit, and grasp correctness
   tests remain green.
10. One natural Gazebo Air-Ground Pick Demo reaches physical `LIFT` using the
    common flight and navigation interfaces and leaves no residual processes.

Only focused interface tests, package builds, and the existing natural E2E are
required. No benchmark, paper-grade evidence, provenance, or new safety
framework is part of acceptance.
