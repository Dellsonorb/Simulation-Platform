#!/bin/bash

set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: validate_p450_current_environment.bash REPOSITORY PX4_ROOT" >&2
  exit 64
fi

p450_repo_root="$(/usr/bin/realpath -e -- "$1")"
p450_supplied_px4="$2"
p450_runtime_config="$p450_repo_root/config/p450_runtime.json"
p450_runtime_validator="$p450_repo_root/tools/p450_runtime.py"

p450_validated_px4="$({
  /usr/bin/env -i PATH=/usr/bin:/bin:/usr/sbin:/sbin \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B "$p450_runtime_validator" \
      --config "$p450_runtime_config" \
      --px4-root "${P450_PX4_ROOT:-}" \
      --repository-root "$p450_repo_root"
})"
p450_expected_px4="$(/usr/bin/realpath -e -- "$p450_supplied_px4")"
if [[ "$p450_validated_px4" != "$p450_expected_px4" || \
    "${P450_PX4_ROOT:-}" != "$p450_expected_px4" ]]; then
  echo "current P450_PX4_ROOT is outside the validated runtime boundary" >&2
  exit 65
fi

p450_install_share="$p450_repo_root/install/p450-clean/share"
p450_prometheus_models="$p450_install_share/prometheus_gazebo/gazebo_models"
p450_overlay_plugins="$p450_repo_root/install/p450-runtime-overlays/lib"
p450_install_plugins="$p450_repo_root/install/p450-clean/lib"
p450_external_plugins="$p450_expected_px4/build/amovlab_sitl_default/build_gazebo"
p450_expected_ros_package_path="$p450_expected_px4:$p450_expected_px4/Tools/sitl_gazebo:$p450_install_share:/opt/ros/noetic/share"
p450_expected_model_path="$p450_install_share/sim_platform_assets/models:$p450_prometheus_models/uav_models:$p450_prometheus_models/sensor_models:$p450_prometheus_models/scene_models:$p450_prometheus_models/r200_models:$p450_prometheus_models/texture:$p450_expected_px4/Tools/sitl_gazebo/models"
if [[ "${ROS_HOME:-}" != "$p450_repo_root/logs/ros" || \
    "${ROS_PACKAGE_PATH:-}" != "$p450_expected_ros_package_path" || \
    "${GAZEBO_MODEL_PATH:-}" != "$p450_expected_model_path" ]]; then
  echo "current ROS/model environment is outside the runtime boundary" >&2
  exit 65
fi

p450_validate_library_path() {
  local p450_value="$1"
  local p450_label="$2"
  local p450_entry=""
  local p450_canonical=""
  local p450_candidate=""
  local p450_index=0
  local p450_livox_hits=0
  local -a p450_entries=()
  local -a p450_prefix=(
    "$p450_overlay_plugins" "$p450_install_plugins" "$p450_external_plugins")
  local -A p450_seen=()

  if [[ -z "$p450_value" || "$p450_value" == :* || \
      "$p450_value" == *: || "$p450_value" == *::* ]]; then
    echo "$p450_label has empty path entries" >&2
    return 65
  fi
  IFS=: read -r -a p450_entries <<< "$p450_value"
  if (( ${#p450_entries[@]} < 3 )); then
    echo "$p450_label omits the canonical plugin prefix" >&2
    return 65
  fi
  for p450_index in 0 1 2; do
    if [[ "${p450_entries[$p450_index]}" != "${p450_prefix[$p450_index]}" ]]; then
      echo "$p450_label has the wrong canonical plugin order" >&2
      return 65
    fi
  done
  for p450_entry in "${p450_entries[@]}"; do
    if [[ "$p450_entry" != /* ]] || \
        ! p450_canonical="$(/usr/bin/realpath -e -- "$p450_entry")" || \
        [[ "$p450_canonical" != "$p450_entry" || ! -d "$p450_entry" || \
           ! -r "$p450_entry" || -n "${p450_seen[$p450_canonical]:-}" ]]; then
      echo "$p450_label has a noncanonical, duplicate, or unreadable entry" >&2
      return 65
    fi
    p450_seen[$p450_canonical]=1
    p450_candidate="$p450_entry/liblivox_laser_gazebo_plugins.so"
    if [[ -e "$p450_candidate" || -L "$p450_candidate" ]]; then
      if [[ "$p450_candidate" != \
          "$p450_install_plugins/liblivox_laser_gazebo_plugins.so" ]] || \
          [[ "$(/usr/bin/realpath -e -- "$p450_candidate")" != \
          "$p450_candidate" || ! -f "$p450_candidate" || \
          ! -r "$p450_candidate" ]]; then
        echo "$p450_label contains a Livox plugin shadow" >&2
        return 65
      fi
      p450_livox_hits=$((p450_livox_hits + 1))
    fi
  done
  if [[ "$p450_livox_hits" != "1" ]]; then
    echo "$p450_label must contain the installed Livox plugin exactly once" >&2
    return 65
  fi
}

p450_validate_library_path "${GAZEBO_PLUGIN_PATH:-}" GAZEBO_PLUGIN_PATH
p450_validate_library_path "${LD_LIBRARY_PATH:-}" LD_LIBRARY_PATH

p450_require_package_path() {
  local p450_package="$1"
  local p450_expected="$2"
  local p450_observed=""
  if ! p450_observed="$(
    /opt/ros/noetic/bin/rospack find "$p450_package" 2>/dev/null
  )" || [[ "$p450_observed" != "$p450_expected" ]]; then
    echo "ROS package $p450_package resolved outside its runtime boundary: ${p450_observed:-missing}" >&2
    return 65
  fi
}

p450_require_package_path px4 "$p450_expected_px4"
p450_require_package_path mavlink_sitl_gazebo \
  "$p450_expected_px4/Tools/sitl_gazebo"
for p450_package in \
    sim_platform_bringup prometheus_msgs realsense_ros_gazebo \
    prometheus_gazebo prometheus_uav_control brick_aerial_perception \
    sim_platform_assets livox_laser_gazebo_plugins; do
  p450_require_package_path "$p450_package" \
    "$p450_install_share/$p450_package"
done
for p450_package in gazebo_ros mavros tf2_ros; do
  p450_require_package_path "$p450_package" \
    "/opt/ros/noetic/share/$p450_package"
done
