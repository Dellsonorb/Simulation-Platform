# M1 P450 Aerial Perception Simulation

This package adds the smallest aerial perception loop on top of the existing
Prometheus P450/PX4 Gazebo stack. It does not replace the flight controller and
does not depend on BUNKER or the Ground-only workspaces.

## Runtime chain

`Prometheus P450_D435i -> fixed viewpoint mission -> synchronized RGB/depth -> depth-to-color registration -> red brick mask -> world point cloud -> top-surface OBB/PCA -> world Brick Pose`

The official simulated D435i publishes color and depth with different camera
intrinsics. Raw depth therefore must not be indexed with a color mask. The M1
node registers raw `16UC1` depth into the color imager and publishes an aligned
`32FC1` image before back-projection.

## Build and launch

```bash
cd /home/lu/navCarProject_ws/p450-sensors
source /opt/ros/noetic/setup.bash
source /home/lu/P450/prometheus/Prometheus/devel/setup.bash --extend
catkin_make --source Modules/brick_aerial_perception \
  --build build/brick_aerial_perception \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

./Modules/brick_aerial_perception/scripts/run_m1_demo.bash
```

The normal demo opens Gazebo and RViz and automatically flies all four views.
For a headless validation report:

```bash
./Modules/brick_aerial_perception/scripts/run_m1_demo.bash \
  gui:=false rviz_enable:=false validation_enable:=true \
  validation_output:=/tmp/m1_validation.json
```

## Interfaces

| Purpose | Topic | Type / frame |
|---|---|---|
| Raw color | `/uav1/camera/color/image_raw` | `sensor_msgs/Image`, `rgb8` |
| Raw depth | `/uav1/camera/depth/image_raw` | `sensor_msgs/Image`, `16UC1` |
| Color intrinsics | `/uav1/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| Depth intrinsics | `/uav1/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| Aligned depth | `/m1_aerial_brick_pose/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`, `32FC1` |
| Brick mask | `/m1_aerial_brick_pose/brick_mask` | `sensor_msgs/Image`, `mono8` |
| Debug RGB | `/m1_aerial_brick_pose/brick_debug_rgb` | `sensor_msgs/Image`, `rgb8` |
| Brick cloud | `/m1_aerial_brick_pose/brick_points` | `sensor_msgs/PointCloud2`, `world` |
| Brick pose | `/m1_aerial_brick_pose/brick_pose` | `geometry_msgs/PoseStamped`, `world` |
| Quality gate | `/m1_aerial_brick_pose/quality_status` | JSON `std_msgs/String` |
| Mission phase | `/m1/viewpoint_state` | `std_msgs/String` |

The unambiguous M1 TF chain is `world -> uav1/base_link_gt ->
uav1/camera_color_optical_frame`. In simulation the dynamic base transform is
fed by `/uav1/prometheus/ground_truth`; this input must be replaced by the
aircraft localization output before real flight.

## Fixed views

The four Prometheus ENU command viewpoints are parameterized in
`config/viewpoints.yaml`:

- `center_low`: `[-1.70, 0.00, 1.35]`, yaw `0°`
- `left_low`: `[-1.70, -0.55, 1.35]`, yaw `+17.9°`
- `right_low`: `[-1.70, +0.55, 1.35]`, yaw `-17.9°`
- `center_high`: `[-2.20, 0.00, 1.75]`, yaw `0°`

Every view must meet position, yaw, linear-speed and angular-speed tolerances
for 1.5 seconds before perception is enabled. Timeout or flight-health loss
causes hover/landing instead of continued sampling.

## Future Ground handoff

The handoff contract is already a `PoseStamped` 4-DoF estimate in the unified
world frame. P450-to-Ground integration only needs a checked `world <-> odom/map`
transform and an adapter/remap to the Ground coarse-pose input; it must also
forward `quality_status` and reject stale poses. No Ground module is started or
modified by this package.
