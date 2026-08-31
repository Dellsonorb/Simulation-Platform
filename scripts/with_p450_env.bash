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
# shellcheck disable=SC2016
exec "$p450_noetic_wrapper" \
  /bin/bash --noprofile --norc -c '
    set -euo pipefail
    p450_repo_root="$1"
    p450_px4_root="$2"
    shift 2

    source "$p450_repo_root/install/p450-clean/setup.bash"

    p450_prometheus_share="$p450_repo_root/install/p450-clean/share/prometheus_gazebo"
    p450_external_models="$p450_px4_root/Tools/sitl_gazebo/models"
    p450_external_plugins="$p450_px4_root/build/amovlab_sitl_default/build_gazebo"

    export P450_PX4_ROOT="$p450_px4_root"
    export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$p450_px4_root:$p450_px4_root/Tools/sitl_gazebo"
    export GAZEBO_MODEL_PATH="$p450_prometheus_share/gazebo_models/uav_models:$p450_prometheus_share/gazebo_models/sensor_models:$p450_prometheus_share/gazebo_models/scene_models:$p450_prometheus_share/gazebo_models/r200_models:$p450_prometheus_share/gazebo_models/texture:$p450_external_models"
    export GAZEBO_PLUGIN_PATH="$p450_external_plugins"
    export LD_LIBRARY_PATH="$p450_external_plugins${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

    exec "$@"
  ' p450-runtime "$p450_repo_root" "$p450_px4_root" "$@"
