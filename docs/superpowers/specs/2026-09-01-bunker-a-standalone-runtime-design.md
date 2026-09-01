# BUNKER-A Standalone Runtime Design

**Status:** Approved for implementation on 2026-09-01

## Purpose

BUNKER-A creates the first independent Ground runtime for Simulation Platform
V1.0. It must provide a bounded, install-space-capable BUNKER model with a
bounded-command, collision-enabled planar interface, odometry, a complete TF
chain, and a 2D LiDAR in Gazebo. It is a platform component that can later be
included in the shared P450 + Ground world; it is not a navigation,
manipulation, or paper-evaluation launch.

The current imported Ground stack does not meet that boundary:

- `bunker_only.launch` spawns a static model and exposes no command, odometry,
  scan, or usable TF data;
- the movable plugin in `bunker_navigation` requires AUBO and AG95 entities and
  also owns gripper settling, collision masks, anchoring, and manipulation
  handoff;
- the imported BUNKER description does not install its URDF or meshes; and
- the existing spawn test can pass without testing motion, topics, TF, LiDAR,
  install closure, or clean shutdown.

## Scope

BUNKER-A must provide:

1. a platform-owned `bunker_sim_runtime` Catkin package;
2. an installed BUNKER visual model derived from the frozen vendor assets;
3. one stable primitive collision proxy and fixed decorative wheel joints;
4. non-holonomic, bounded `/ground/cmd_vel` admission;
5. Gazebo planar motion with a 0.5 second command watchdog;
6. `/ground/odom`, `/ground/scan`, and a connected, uniquely-owned TF tree;
7. a worldless component launch and a standalone diagnostic world launch;
8. offline source/install contract tests; and
9. a bounded live smoke that writes an atomic PASS/FAIL result and leaves no
   ROS or Gazebo processes behind.

The following remain outside this milestone:

- `move_base`, DWA, heading gates, approach-pose logic, maps, or localization;
- AUBO, AG95, D435, MoveIt, grasping, attachment, or navigation-to-pick
  handoff;
- physical track tension, sprocket dynamics, slip calibration, slope claims,
  or sim-to-real locomotion metrics;
- any benchmark, scenario matrix, Pilot/T2/Formal evidence, RBP, M5,
  Task-aware method, or paper report; and
- changes to `/media/lu/P450_PAPER/P450-PAPER` or any external PX4/source
  checkout.

## Dependency Boundary

The new runtime has this one-way dependency graph:

```text
vendor/bunker_description
          |
          v
platform/bunker_sim_runtime
          |
          +-- gazebo_ros + gazebo_plugins
          +-- robot_state_publisher + tf2_ros
          +-- rospy + xacro
          +-- geometry_msgs + nav_msgs + sensor_msgs
```

It must not depend on any of these packages:

```text
bunker_aubo_description  bunker_aubo_gazebo  bunker_navigation
aubo_description         dh_ag95_description bunker_aubo_moveit_config
brick_*                  ground_pick_orchestrator
move_base                controller_manager
```

Consumers may include `bunker_sim_runtime`; the runtime must never include a
consumer. Existing imported Ground launches remain classified as legacy
component diagnostics until they are deliberately migrated in later
milestones.

The package manifest has one `catkin` build-tool dependency. Its runtime
dependencies are exactly `bunker_description`, `gazebo_plugins`, `gazebo_ros`,
`geometry_msgs`, `nav_msgs`, `robot_state_publisher`, `rospy`, `roslaunch`,
`sensor_msgs`, `tf2_ros`, and `xacro`. Its test-only dependencies are
`gazebo_msgs`, `liburdfdom-tools`, `rosgraph`, `rosgraph_msgs`, `rostest`,
`rosunit`, and `tf2_msgs`. The smoke may use Python standard-library modules
and Gazebo command-line tools; it must not gain a dependency on an imported
Ground consumer merely to reuse a helper.

## Vendor Asset and Install Boundary

The vendor BUNKER URDF and meshes remain byte-for-byte imported source assets.
The only vendor-package repair in this milestone is an install rule in
`src/vendor/bunker_description/CMakeLists.txt` for `meshes` and `urdf`; legacy
vendor launch, RViz, and configuration surfaces are deliberately not
installed. That SIM-local change must be recorded with its final SHA-256 in
`config/runtime_overlay.json`; the external source repository must remain
clean and unchanged.

`config/runtime_sources.json` must register `bunker_sim_runtime` as a
non-imported package at `src/platform/bunker_sim_runtime`. An installed asset
manifest must record every installed vendor URDF and all 22 installed meshes
with relative paths and exact source/install SHA-256, separately identify the
runtime-referenced mesh subset, reject any symlink or canonical path escaping
either package tree, and explicitly preserve the upstream
`bunker_description/package.xml` license value `TODO`. The install repair does
not assert that this unresolved upstream redistribution license has been
settled.

An install-only probe starts through `env -i`, sources only
`/opt/ros/noetic/setup.bash`, `/usr/share/gazebo/setup.sh`, and
`install/p450-clean/setup.bash`, and rejects source/devel paths in every ROS,
Python, CMake, Gazebo, or loader search variable. In that environment,
`rospack find bunker_description` and `rospack find bunker_sim_runtime` must
both resolve below `install/p450-clean`, and all rendered mesh URIs must
resolve to regular installed files whose hashes match the manifest. The
historical `p450-clean` Catkin profile remains the build profile for this
milestone so the already-verified P450 runtime is not migrated merely to
rename an install space.

The probe also resolves `libgazebo_ros_planar_move.so` and
`libgazebo_ros_laser.so` exactly below `/opt/ros/noetic/lib`, and Gazebo's
`libRayPlugin.so` exactly below
`/usr/lib/x86_64-linux-gnu/gazebo-11/plugins`. It initializes variables needed
by `/usr/share/gazebo/setup.sh` before sourcing it under strict-shell mode.
The live gate confirms the same canonical files in `/proc/<gzserver>/maps` and
rejects a same-basename plugin loaded from any other directory.

## Runtime Robot Rendering

`bunker_sim_runtime` owns a deterministic Python renderer. Its only input is
the installed `bunker_description/urdf/bunker.urdf.xacro`, frozen at SHA-256
`41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74`.
The higher-level vendor `bunker.xacro` is forbidden because it already adds a
planar plugin. The renderer runs the installed input through xacro, parses the
result, validates an input of exactly 17 links, 16 joints, and zero plugins,
and emits the runtime URDF. It performs only the following declared
transformations:

1. map each source link, joint, reference, and generated Gazebo entity name to
   `[A-Za-z][A-Za-z0-9_]*` by replacing every maximal invalid run with one
   underscore, then reject empty names or any collision in the mapped names;
2. convert all 16 `wheel*` joints to fixed joints and remove incompatible
   axis, limit, dynamics, calibration, mimic, and safety-controller elements;
3. remove every vendor mesh collision while retaining every visual and
   inertial element;
4. add one `base_link` box collision with:
   - origin `0.018058912 0.001357451 -0.160420741` metres;
   - size `1.026335219 0.782744936 0.395154782` metres;
5. add `lidar_2d_link` and a fixed `lidar_2d_joint` at
   `[-0.30, 0.0, 0.25]` metres with zero rotation;
6. add exactly one Gazebo ray sensor and one
   `libgazebo_ros_laser.so` plugin; and
7. add exactly one `libgazebo_ros_planar_move.so` plugin.

The box values are the AABB of the transformed vendor `BUNKER.STL`. Replacing
approximately 512,000 collision triangles with one explicit proxy preserves
the visual model while giving ODE a stable and understandable contact shape.
The runtime URDF must have one root (`base_link`), 18 links, 17 fixed joints,
one collision, one LiDAR sensor, and exactly two plugin elements: one planar
model plugin and one laser sensor plugin. Their libraries and names must be
unique, and no other input or output plugin is permitted. It must contain no
arm, gripper, camera, controller, attachment, handoff, navigation, or brick
entity.

The renderer is installed as the executable
`share/bunker_sim_runtime/scripts/render_bunker_runtime.py`, not through
`catkin_install_python`, and the launch invokes that exact installed path.
The velocity guard is a ROS node installed through `catkin_install_python` and
is resolved by roslaunch as a package executable. Install-only tests reject a
source-tree or devel-space renderer invocation.

## Motion Semantics

The installed Gazebo 11 / `gazebo_plugins` 2.9.3 planar plugin is used instead
of copying the legacy Ground plugin. This plugin applies model linear and
angular velocity through Gazebo, publishes odometry and TF, owns a callback
queue, and supports `cmdTimeout`.

The model plugin configuration is fixed to:

```text
robotNamespace  /ground
commandTopic    cmd_vel_safe
odometryTopic   odom
odometryFrame   odom
robotBaseFrame  base_link
odometryRate    50.0 Hz
cmdTimeout      0.5 s
```

These are the exact SDF tags supported by local `gazebo_plugins` 2.9.3. There
is no `publishTF` setting: the plugin always owns and publishes the dynamic
`odom -> base_link` edge. `tf_prefix` is not an SDF plugin tag. Before Gazebo
loads either plugin, launch sets the ROS parameter `/ground/tf_prefix` to
`ground`; the planar plugin keeps the relative frame names shown above and
resolves them to `ground/odom` and `ground/base_link` through that parameter.

The plugin receives only guarded commands. A platform-owned velocity guard
subscribes to `/ground/cmd_vel` and publishes `/ground/cmd_vel_safe`:

- any NaN or infinity in any Twist component is rejected and immediately
  produces a zero Twist;
- nonzero `linear.y`, `linear.z`, `angular.x`, or `angular.y` above `1e-9` is
  rejected and immediately produces a zero Twist;
- valid `linear.x` is clamped to `[-0.5, 0.5]` m/s;
- valid `angular.z` is clamped to `[-1.0, 1.0]` rad/s; and
- queue depths are one so stale commands cannot accumulate.

The guard is a required roslaunch node: unexpected guard exit terminates the
component launch, and the live monitor requires it to remain present until
controlled shutdown. It does not implement a lifecycle or duplicate the
watchdog. If valid commands stop, the Gazebo plugin's 0.5 second timeout is the
sole motion watchdog and must drive both linear and angular velocity to zero.
The timeout uses ROS time, so the smoke proves `/clock` advances by more than
0.5 simulated seconds during the timeout phase. `/use_sim_time` is frozen to
the boolean value `true`; a wall-clock sleep or a merely-present `/clock`
topic alone is not watchdog evidence.

This is deliberately described as a bounded-command, collision-enabled
kinematic planar interface, not as a physical tracked-vehicle model. The
plugin writes model velocity directly on every world update and provides no
braking, acceleration, traction, slip, or collision-safety guarantee. This
milestone proves that one declared collision proxy is materialized and loaded,
not that contact response prevents penetration. It proves ROS/Gazebo
interfaces, command admission, bounded planar motion, TF, and sensor
integration; it does not prove track dynamics, collision safety, or odometry
noise realism.

## ROS Graph and Namespace Contract

The V1 instance identity is frozen to namespace `/ground`, model name
`bunker`, and frame prefix `ground`. No launch argument may override any of
those three values; a later multi-Ground milestone requires a separate design
instead of partially templating this one. The default spawn pose is
`[0.0, 0.0, 0.36, 0.0, 0.0, 0.0]` in
`[x, y, z, roll, pitch, yaw]` order. Worldless launch may override only the six
pose scalars, and each must parse as a finite decimal before spawn.

The renderer and launch validators nevertheless reject any externally supplied
model/entity name not matching `[a-z][a-z0-9_]{0,63}`. Local frame/link/joint
names must match `[A-Za-z][A-Za-z0-9_]{0,63}`. Dots, `..`, slashes, whitespace,
empty strings, and XML/ROS special characters are never accepted as identity
input. The ROS namespace is not accepted as free text; it is the literal
`/ground` derived from the frozen identity.

| Resource | Owner | Contract |
|---|---|---|
| `/ground/robot_description` | launch | rendered installed runtime URDF |
| `/ground/cmd_vel` | external consumer | public `geometry_msgs/Twist` input |
| `/ground/cmd_vel_safe` | velocity guard | internal guarded Twist |
| `/ground/odom` | `/gazebo` | `nav_msgs/Odometry`, 50 Hz target |
| `/ground/scan` | `/gazebo` | `sensor_msgs/LaserScan`, 15 Hz target |
| `world -> ground/odom` | `/ground/world_to_odom` | static identity |
| `ground/odom -> ground/base_link` | `/gazebo` | current-time dynamic transform |
| `ground/base_link -> ground/lidar_2d_link` | `/ground/robot_state_publisher` | exact fixed mount |

All URDF link names and plugin frame inputs remain unprefixed local names.
Launch sets `/ground/tf_prefix=ground`; both Gazebo plugins and the namespaced
ROS 1 `robot_state_publisher` read that same parameter. The laser plugin input
is `frameName=lidar_2d_link`, never the already-resolved
`ground/lidar_2d_link`, so the prefix cannot be doubled. The world edge is
published by tf2's quaternion-form static publisher with arguments
`0 0 0 0 0 0 1 world ground/odom`, under namespace/name
`/ground/world_to_odom`. Offline tests exercise this exact local Noetic prefix
mechanism before the live TF authority gate.

The full TF tree is therefore:

```text
world
  `-- ground/odom
        `-- ground/base_link
              |-- ground/lidar_2d_link
              `-- ground/<fixed visual wheel frames>
```

Each edge must have exactly one authority. No unprefixed `odom`, `base_link`,
or LiDAR frame may appear on `/tf` or `/tf_static`.

## LiDAR Contract

The 2D LiDAR is enabled by default because it is part of the requested Ground
platform, not a navigation-method option. Its fixed configuration is:

```text
topic            /ground/scan
plugin frameName lidar_2d_link (resolved through /ground/tf_prefix)
update rate      15.0 Hz
horizontal rays  720
ray resolution   1
minimum angle    -pi
maximum angle    +pi
minimum range    0.12 m
maximum range    8.0 m
range resolution 0.01 m
Gaussian sigma   0.002 m
```

`always_on` and `update_rate` are sensor elements. Samples, horizontal
resolution, angles, ranges, and Gaussian `mean=0`/`stddev=0.002` are ray
elements. The laser plugin itself contains only the supported
`robotNamespace`, `topicName`, and relative `frameName`; unsupported plugin
copies of update rate or Gaussian noise are forbidden. With 720 samples and
ray resolution one, Gazebo 11 publishes 720 ranges and an angle increment of
`2*pi/(720-1)`, approximately `0.008738783459` rad before float32
serialization. This plugin publishes `scan_time=0` and `time_increment=0`.
The `0.01 m` ray range resolution has no `LaserScan` field and is therefore an
offline rendered-SDF/URDF contract, not a fabricated live-message assertion.

The standalone world contains ground, sun, and one fixed box obstacle centred
at `[2.0, 0.0, 0.5]` metres with size `[0.5, 1.0, 1.0]` metres. This guarantees
finite returns without coupling the sensor gate to navigation or mapping.

## Launch Boundaries

Two launch surfaces are required:

1. `bunker_runtime.launch` is worldless. It renders the model, starts the
   required velocity guard, spawns exactly one Gazebo model, starts one
   namespaced robot-state publisher, and publishes the static world-to-odom
   edge. It assumes a Gazebo world already exists and is the launch included
   later by shared-world bringup. Before model spawn or plugin load, its spawn
   preflight requires the existing global `/use_sim_time` parameter to be
   boolean `true`, or the launch fails closed. It sets
   `/ground/tf_prefix=ground` before plugin/model startup.
2. `bunker_standalone.launch` starts the diagnostic world with `gui:=false` by
   default, explicitly passes `use_sim_time:=true` to Gazebo with no public
   override, and includes `bunker_runtime.launch` exactly once.

Neither launch may start RViz, move_base, joint/controller managers, AUBO,
AG95, D435, manipulation nodes, or orchestration. Spawn completion is not
readiness; the live gate waits on model, plugin subscriber/publishers, data,
and TF conditions.

## Clean Runtime Environment

Every install-only or live invocation starts in `env -i` and then explicitly
constructs its environment; it never inherits an ambient workspace overlay.
It sources only the three approved setup files named in the install boundary
and pins `PATH`, locale, the SIM-local install prefix, and the canonical Gazebo
11 plugin directory. Before sourcing Gazebo under `set -u`, its expected
variables are initialized to empty values. The wrapper rejects any final
`ROS_PACKAGE_PATH`, `CMAKE_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`,
`GAZEBO_PLUGIN_PATH`, `GAZEBO_MODEL_PATH`, or `GAZEBO_RESOURCE_PATH` entry that
resolves into a source/devel tree or outside the approved `/opt`, `/usr`, and
SIM-local install/run roots.

Each smoke creates one private mode-0700 run directory below
`/media/lu/P450_PAPER/SIM/p450_sim_v1/logs/bunker_standalone`. It assigns
run-local `TMPDIR`, `ROS_HOME`, `ROS_LOG_DIR`, `GAZEBO_LOG_PATH`,
`IGN_FUEL_CACHE_PATH`, `XDG_CACHE_HOME`, `XDG_CONFIG_HOME`, and
`XDG_DATA_HOME`; it does not repurpose `HOME`. `GAZEBO_MODEL_DATABASE_URI` is
empty, online model lookup is forbidden, and all model/resource paths are
explicit. The result records the sanitized environment and canonical package,
model, mesh, and loaded-plugin paths. Any detected write outside
`/media/lu/P450_PAPER/SIM` and the run-local temporary tree is a failure.

## Offline Acceptance

Offline tests must execute behavior rather than freeze only source strings.
They must prove:

- the package dependency closure contains only the approved boundary;
- the frozen `/ground`/`bunker`/`ground` identity has no override surface,
  pose scalars reject non-finite values, and name validators reject every
  non-canonical grammar case;
- the renderer rejects unexpected vendor structure, duplicate names, missing
  meshes, symlinks escaping the install tree, a wrong frozen input hash, any
  input plugin, extra output plugins, and malformed XML;
- rendered URDF counts, fixed joints, collision proxy, LiDAR values, plugin
  parameters, and forbidden-token absence are exact;
- the velocity guard rejects non-finite and lateral/out-of-plane commands,
  clamps legal axes, and emits zero for rejected input;
- worldless and standalone launch graphs contain the exact expected nodes,
  the guard is required, and the local Noetic `tf_prefix` resolution produces
  each expected resolved frame exactly once without double prefixes;
- install-only `rospack`, xacro/rendering, mesh URI resolution, XML parsing,
  `check_urdf`, canonical plugin resolution, and package-executable resolution
  work in the sanitized environment without source/devel paths;
- source/install asset hashes and symlink closure match, the runtime source
  ledger registers the platform package, and the overlay ledger reflects the
  SIM-local vendor install repair and unresolved upstream license marker;
  and
- existing P450 D435 and MID360 offline contracts still pass.

## Bounded Live Acceptance

`scripts/smoke_bunker_standalone.bash` owns an isolated ROS master, Gazebo
master, sanitized environment, private 0700 run directory, bounded process
group, and atomic schema-2 `result.json`. A PASS requires all of the following
in one fresh run:

1. exactly one live `gzserver`, one `/gazebo` node, one required
   `/ground/velocity_guard` node, and one `bunker` model;
2. with transient probe subscribers disconnected, `/ground/cmd_vel` has
   exactly one subscriber, the guard; `/ground/cmd_vel_safe` has exactly one
   publisher, the guard, and exactly one subscriber, `/gazebo`;
3. `/ground/odom` and `/ground/scan` each have exactly one `/gazebo`
   publisher and the resolved message types are exactly `nav_msgs/Odometry`
   and `sensor_msgs/LaserScan`;
4. loaded `libgazebo_ros_planar_move.so`, `libgazebo_ros_laser.so`, and
   `libRayPlugin.so` paths in the live `gzserver` maps equal the canonical
   paths declared in the clean-runtime contract, with no shadow copy;
5. `/use_sim_time` is present with type boolean and value `true`; at least 20
   initial odometry and eight scan samples have nonzero, strictly increasing
   `/clock`-based simulation timestamps and exact frame contracts. Median
   simulation-time periods are `0.016-0.030 s` for odometry and
   `0.050-0.090 s` for scans;
6. every scan has exactly 720 ranges and 720 finite intensities. Frame ID is
   exactly `ground/lidar_2d_link`; `angle_min`, `angle_max`,
   `angle_increment=2*pi/719`, `range_min`, and `range_max` match the contract
   within `1e-6`; `time_increment` and `scan_time` match zero within `1e-9`.
   Every range is either finite inside the declared inclusive range interval
   (with `1e-6` comparison tolerance) or positive infinity; NaN, negative
   infinity, and out-of-range finite values fail. At least one return is
   finite and in range;
7. live command admission is exercised before the motion baseline: one
   over-limit command is observed on the safe topic clamped to
   `linear.x=0.5` and `angular.z=1.0`; one lateral/out-of-plane command and one
   non-finite command each produce an all-zero safe Twist within `0.10`
   simulated seconds. The transient safe-topic probe then disconnects, an
   explicit zero is sent, watchdog stop is observed, and the graph ownership
   assertion is repeated;
8. all `/clock`, model pose/twist, odometry pose/twist, scan metadata, and
   relevant transform values required by the gate are finite, except the
   explicitly permitted positive-infinity scan misses;
9. a `0.25 m/s` forward command published at 20 Hz for 2.0 simulated seconds,
   followed by watchdog stop, produces `0.50-0.70 m` total forward
   displacement and less than `0.10 m` lateral error, with both deltas projected
   into the settled pre-command baseline-yaw frame rather than assumed world
   axes. Gazebo and odometry are compared only in x/y/yaw and agree within
   `0.03 m` in the plane;
10. from the new settled pose, a `0.5 rad/s` rotation command published at
    20 Hz for 1.5 simulated seconds, followed by watchdog stop, produces
    `0.85-1.15 rad` unwrapped yaw change, less than `0.08 m` planar drift, and
    Gazebo/odometry yaw agreement within `0.04 rad`;
11. for each command phase, `/clock` advances by at least the 0.5-second
    timeout after the final command, the time-to-stop and watchdog coast are
    recorded, and both Gazebo model-state twist and odometry twist fall below
    `0.01` linear and angular speed. Only after that observed stop begins an
    independent one-simulated-second stationary window; additional planar
    drift and unwrapped yaw drift are each below `0.01 m` and `0.01 rad`;
12. `world -> ground/odom -> ground/base_link -> ground/lidar_2d_link` is
    current and connected, each edge has exactly one expected authority, the
    LiDAR mount matches the declared transform within `1e-6`, and no
    unprefixed or double-prefixed runtime frame exists;
13. live Gazebo model inspection contains exactly the declared single box
    collision, with pose and size matching the rendered contract within
    `1e-6`, and no mesh collision. This proves collision-proxy
    materialization/loading only. Model height remains within `0.03 m` of its
    settled baseline and absolute roll/pitch remain below `0.05 rad`;
14. the launch log contains no segmentation fault, assertion, abort, core
    dump, `boost thread: trying joining itself`, unexpected process death,
    plugin/dependency-load failure, or XML/mesh error;
15. controlled SIGINT returns launcher status zero, needs no TERM/KILL
    escalation, and leaves no ROS/Gazebo process or owned port; and
16. result evidence records the sanitized environment, package/plugin/model
    paths, installed input/model/mesh hashes, graph owners, sample/rate
    summaries, guard input/output probes, planar poses, TF authorities,
    command-window and watchdog-coast deltas, both twist sources,
    script/launcher status, and shutdown escalation.

The smoke must fail closed when any evidence is missing. A rostest XML summary
cannot override a fatal process log or a non-clean shutdown.

Because BUNKER is installed into the same historical Catkin space as the P450
runtime, final review also requires fresh zero-exit D435 and MID360 standalone
smokes after the BUNKER live PASS. Those regressions do not make BUNKER depend
on PX4; they prove that expanding the common install space did not damage the
already-accepted Air runtime.

## Integration Contract for Later Milestones

The shared-world bringup will include `bunker_runtime.launch`, not
`bunker_standalone.launch`. AUBO/AG95 integration will consume the same
`ground/base_link` and must not replace the base drive, odometry, LiDAR, or TF
authorities. Navigation, if reintroduced, will publish only to
`/ground/cmd_vel`; manipulation will begin only after the same watchdog-backed
base is stopped. This keeps base motion, arm control, sensing, and mission
orchestration independently testable.

## Failure Handling

- Invalid pose, frozen identity, renderer input, or asset/plugin provenance
  exits before spawning Gazebo.
- Invalid velocity input produces an immediate zero command and a throttled
  error; it is never forwarded partially.
- Unexpected velocity-guard exit terminates the component launch; no consumer
  is allowed to publish directly to the safe topic.
- Missing assets, plugins, topics, publishers, TF edges, or samples are
  permanent live-gate failures, not warnings.
- A missing/non-boolean/false `/use_sim_time` or paused/stalled `/clock` is a
  watchdog-contract failure, even if wall time continues to pass.
- Startup uses condition-based waits with explicit deadlines; it never assumes
  `spawn_model` completion means plugin readiness.
- Cleanup always preserves the original failure status and records whether it
  needed SIGINT, TERM, or KILL.
- Every path and environment preflight happens before launch; no success or
  failure path may write outside `/media/lu/P450_PAPER/SIM` or modify an
  external repository.
