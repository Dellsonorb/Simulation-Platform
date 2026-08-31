#!/bin/bash

set -euo pipefail

if [[ $# -eq 0 ]]; then
  echo "usage: P450_PX4_ROOT=/path scripts/with_p450_env.bash COMMAND [ARG ...]" >&2
  exit 64
fi

if [[ -z "${P450_PX4_ROOT:-}" ]]; then
  echo "P450_PX4_ROOT must name the pinned external PX4 checkout" >&2
  exit 64
fi

p450_gazebo_display="${P450_GAZEBO_DISPLAY:-}"
p450_gazebo_xauthority="${P450_GAZEBO_XAUTHORITY:-}"
if [[ -n "$p450_gazebo_display" || -n "$p450_gazebo_xauthority" ]]; then
  if [[ -z "$p450_gazebo_display" || -z "$p450_gazebo_xauthority" ]]; then
    echo "P450_GAZEBO_DISPLAY and P450_GAZEBO_XAUTHORITY must be set together" >&2
    exit 64
  fi
  if [[ ! "$p450_gazebo_display" =~ ^:[0-9]+([.][0-9]+)?$ ]]; then
    echo "P450_GAZEBO_DISPLAY must name a local X display like :0 or :0.0" >&2
    exit 64
  fi
  if ! p450_gazebo_xauthority="$({
    /usr/bin/realpath -e -- "$p450_gazebo_xauthority"
  } 2>/dev/null)" || [[ ! -f "$p450_gazebo_xauthority" || \
      ! -r "$p450_gazebo_xauthority" ]]; then
    echo "P450_GAZEBO_XAUTHORITY must name a readable regular file" >&2
    exit 66
  fi
fi

p450_wrapper_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
p450_script_dir="${p450_wrapper_path%/*}"
p450_repo_root="${p450_script_dir%/*}"
p450_runtime_config="$p450_repo_root/config/p450_runtime.json"
p450_runtime_validator="$p450_repo_root/tools/p450_runtime.py"
p450_noetic_wrapper="$p450_repo_root/scripts/with_noetic_env.bash"
p450_install_setup="$p450_repo_root/install/p450-clean/setup.bash"

for p450_required_file in \
  "$p450_runtime_config" \
  "$p450_runtime_validator" \
  "$p450_noetic_wrapper" \
  "$p450_install_setup"; do
  if [[ ! -f "$p450_required_file" ]]; then
    echo "P450 runtime prerequisite is missing: $p450_required_file" >&2
    exit 66
  fi
done

if [[ ! -x "$p450_noetic_wrapper" ]]; then
  echo "Noetic environment wrapper is not executable: $p450_noetic_wrapper" >&2
  exit 66
fi

p450_px4_root=""
if p450_px4_root="$(
  /usr/bin/env -i \
    PATH=/usr/bin:/bin:/usr/sbin:/sbin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B "$p450_runtime_validator" \
      --config "$p450_runtime_config" \
      --px4-root "$P450_PX4_ROOT" \
      --repository-root "$p450_repo_root"
)"; then
  :
else
  p450_validation_status="$?"
  exit "$p450_validation_status"
fi

if [[ -z "$p450_px4_root" || "$p450_px4_root" != /* || \
      "$p450_px4_root" != "$(/usr/bin/realpath -e -- "$p450_px4_root")" ]]; then
  echo "runtime validator did not return one canonical P450_PX4_ROOT" >&2
  exit 65
fi

p450_prometheus_share="$p450_repo_root/install/p450-clean/share/prometheus_gazebo"
p450_model_roots=(
  "$p450_prometheus_share/gazebo_models/uav_models"
  "$p450_prometheus_share/gazebo_models/sensor_models"
  "$p450_prometheus_share/gazebo_models/scene_models"
  "$p450_prometheus_share/gazebo_models/r200_models"
  "$p450_prometheus_share/gazebo_models/texture"
)
for p450_model_root in "${p450_model_roots[@]}"; do
  if [[ ! -d "$p450_model_root" ]]; then
    echo "installed Prometheus Gazebo model root is missing: $p450_model_root" >&2
    exit 66
  fi
done

# Variables in the literal below are intentionally expanded only by the child.
# shellcheck disable=SC1004,SC2016
exec "$p450_noetic_wrapper" \
  /bin/bash --noprofile --norc -c '
    set -euo pipefail
    p450_repo_root="$1"
    p450_px4_root="$2"
    p450_gazebo_display="$3"
    p450_gazebo_xauthority="$4"
    shift 4

    source "$p450_repo_root/install/p450-clean/setup.bash"

    p450_prometheus_share="$p450_repo_root/install/p450-clean/share/prometheus_gazebo"
    p450_external_models="$p450_px4_root/Tools/sitl_gazebo/models"
    p450_external_plugins="$p450_px4_root/build/amovlab_sitl_default/build_gazebo"
    p450_local_plugins="$p450_repo_root/install/p450-runtime-overlays/lib"

    export P450_PX4_ROOT="$p450_px4_root"
    if [[ -n "$p450_gazebo_display" ]]; then
      export DISPLAY="$p450_gazebo_display"
      export XAUTHORITY="$p450_gazebo_xauthority"
    fi
    export ROS_PACKAGE_PATH="$p450_px4_root:$p450_px4_root/Tools/sitl_gazebo${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
    export GAZEBO_MODEL_PATH="$p450_prometheus_share/gazebo_models/uav_models:$p450_prometheus_share/gazebo_models/sensor_models:$p450_prometheus_share/gazebo_models/scene_models:$p450_prometheus_share/gazebo_models/r200_models:$p450_prometheus_share/gazebo_models/texture:$p450_external_models"
    export GAZEBO_PLUGIN_PATH="$p450_local_plugins:$p450_external_plugins"
    export LD_LIBRARY_PATH="$p450_local_plugins:$p450_external_plugins${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

    p450_require_package_path() {
      local p450_package="$1"
      local p450_expected="$2"
      local p450_observed
      if ! p450_observed="$(/opt/ros/noetic/bin/rospack find "$p450_package" 2>/dev/null)" ||
          [[ "$p450_observed" != "$p450_expected" ]]; then
        echo "ROS package $p450_package resolved outside its runtime boundary: ${p450_observed:-missing}" >&2
        return 65
      fi
    }

    p450_require_package_path px4 "$p450_px4_root"
    p450_require_package_path mavlink_sitl_gazebo \
      "$p450_px4_root/Tools/sitl_gazebo"
    for p450_package in \
        sim_platform_bringup prometheus_msgs realsense_ros_gazebo \
        prometheus_gazebo prometheus_uav_control brick_aerial_perception; do
      p450_require_package_path "$p450_package" \
        "$p450_repo_root/install/p450-clean/share/$p450_package"
    done
    for p450_package in gazebo_ros mavros tf2_ros; do
      p450_require_package_path "$p450_package" \
        "/opt/ros/noetic/share/$p450_package"
    done

    exec "$@"
  ' p450-runtime "$p450_repo_root" "$p450_px4_root" \
    "$p450_gazebo_display" "$p450_gazebo_xauthority" "$@"
