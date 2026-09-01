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
p450_current_environment_validator="$p450_repo_root/scripts/validate_p450_current_environment.bash"
p450_install_setup="$p450_repo_root/install/p450-clean/setup.bash"

for p450_required_file in \
  "$p450_runtime_config" \
  "$p450_runtime_validator" \
  "$p450_noetic_wrapper" \
  "$p450_current_environment_validator" \
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
if [[ ! -x "$p450_current_environment_validator" ]]; then
  echo "current environment validator is not executable: $p450_current_environment_validator" >&2
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

p450_assets_models="$p450_repo_root/install/p450-clean/share/sim_platform_assets/models"
p450_assets_models_canonical=""
if ! p450_assets_models_canonical="$(
  /usr/bin/realpath -e -- "$p450_assets_models"
)" || [[ "$p450_assets_models_canonical" != "$p450_assets_models" || \
    ! -d "$p450_assets_models" || ! -r "$p450_assets_models" ]]; then
  echo "installed MID360 model root is missing, unreadable, or escapes its canonical path: $p450_assets_models" >&2
  exit 66
fi

p450_livox_plugin="$p450_repo_root/install/p450-clean/lib/liblivox_laser_gazebo_plugins.so"
p450_livox_plugin_canonical=""
if ! p450_livox_plugin_canonical="$(
  /usr/bin/realpath -e -- "$p450_livox_plugin"
)" || [[ "$p450_livox_plugin_canonical" != "$p450_livox_plugin" || \
    ! -f "$p450_livox_plugin" || ! -r "$p450_livox_plugin" ]]; then
  echo "installed Livox plugin is missing, unreadable, or escapes its canonical path: $p450_livox_plugin" >&2
  exit 66
fi

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
    p450_assets_models="$p450_repo_root/install/p450-clean/share/sim_platform_assets/models"
    p450_external_models="$p450_px4_root/Tools/sitl_gazebo/models"
    p450_external_plugins="$p450_px4_root/build/amovlab_sitl_default/build_gazebo"
    p450_local_plugins="$p450_repo_root/install/p450-runtime-overlays/lib"
    p450_install_plugins="$p450_repo_root/install/p450-clean/lib"
    p450_clean_gazebo_plugin_path="${GAZEBO_PLUGIN_PATH:-}"
    p450_clean_ld_library_path="${LD_LIBRARY_PATH:-}"

    p450_append_unique_directory() {
      local p450_array_name="$1"
      local p450_candidate="$2"
      local p450_label="$3"
      local p450_candidate_canonical=""
      local p450_existing=""
      local p450_existing_canonical=""
      local -n p450_entries="$p450_array_name"

      if [[ -z "$p450_candidate" || "$p450_candidate" != /* ]]; then
        echo "$p450_label must contain absolute non-empty directories" >&2
        return 65
      fi
      if ! p450_candidate_canonical="$(
        /usr/bin/realpath -e -- "$p450_candidate"
      )" || [[ ! -d "$p450_candidate" || ! -r "$p450_candidate" ]]; then
        echo "$p450_label directory is missing or unreadable: $p450_candidate" >&2
        return 66
      fi
      for p450_existing in "${p450_entries[@]}"; do
        if ! p450_existing_canonical="$(
          /usr/bin/realpath -e -- "$p450_existing"
        )"; then
          echo "$p450_label directory became unavailable: $p450_existing" >&2
          return 66
        fi
        if [[ "$p450_existing_canonical" == "$p450_candidate_canonical" ]]; then
          return 0
        fi
      done
      p450_entries+=("$p450_candidate")
    }

    p450_append_directory_list() {
      local p450_array_name="$1"
      local p450_path_list="$2"
      local p450_label="$3"
      local p450_entry=""
      local -a p450_list_entries=()

      if [[ -z "$p450_path_list" ]]; then
        return 0
      fi
      if [[ "$p450_path_list" == :* || "$p450_path_list" == *: || \
          "$p450_path_list" == *::* ]]; then
        echo "$p450_label must contain absolute non-empty directories" >&2
        return 65
      fi
      IFS=: read -r -a p450_list_entries <<< "$p450_path_list"
      for p450_entry in "${p450_list_entries[@]}"; do
        p450_append_unique_directory \
          "$p450_array_name" "$p450_entry" "$p450_label"
      done
    }

    p450_join_directories() {
      local p450_array_name="$1"
      local -n p450_entries="$p450_array_name"
      local IFS=:
      printf "%s" "${p450_entries[*]}"
    }

    p450_require_unshadowed_livox() {
      local p450_path_list="$1"
      local p450_label="$2"
      local p450_entry=""
      local p450_candidate=""
      local p450_candidate_canonical=""
      local p450_livox_hits=0
      local -a p450_entries=()

      IFS=: read -r -a p450_entries <<< "$p450_path_list"
      for p450_entry in "${p450_entries[@]}"; do
        p450_candidate="$p450_entry/liblivox_laser_gazebo_plugins.so"
        if [[ ! -e "$p450_candidate" && ! -L "$p450_candidate" ]]; then
          continue
        fi
        if ! p450_candidate_canonical="$(
          /usr/bin/realpath -e -- "$p450_candidate"
        )" || [[ ! -f "$p450_candidate" || ! -r "$p450_candidate" ]]; then
          echo "$p450_label has an invalid Livox plugin candidate: $p450_candidate" >&2
          return 66
        fi
        if [[ "$p450_candidate" != "$p450_livox_plugin" || \
            "$p450_candidate_canonical" != "$p450_livox_plugin_canonical" ]]; then
          echo "$p450_label has a Livox plugin shadow: $p450_candidate" >&2
          return 65
        fi
        p450_livox_hits=$((p450_livox_hits + 1))
      done
      if [[ "$p450_livox_hits" -lt 1 ]]; then
        echo "$p450_label does not contain the installed Livox plugin" >&2
        return 66
      fi
    }

    p450_livox_plugin="$p450_install_plugins/liblivox_laser_gazebo_plugins.so"
    p450_livox_plugin_canonical="$(
      /usr/bin/realpath -e -- "$p450_livox_plugin"
    )"
    p450_gazebo_plugin_entries=()
    p450_ld_library_entries=()
    for p450_plugin_root in \
        "$p450_local_plugins" "$p450_install_plugins" \
        "$p450_external_plugins"; do
      p450_append_unique_directory \
        p450_gazebo_plugin_entries "$p450_plugin_root" GAZEBO_PLUGIN_PATH
      p450_append_unique_directory \
        p450_ld_library_entries "$p450_plugin_root" LD_LIBRARY_PATH
    done
    p450_append_directory_list \
      p450_gazebo_plugin_entries "$p450_clean_gazebo_plugin_path" \
      GAZEBO_PLUGIN_PATH
    p450_append_directory_list \
      p450_ld_library_entries "$p450_clean_ld_library_path" LD_LIBRARY_PATH
    p450_final_gazebo_plugin_path="$(
      p450_join_directories p450_gazebo_plugin_entries
    )"
    p450_final_ld_library_path="$(
      p450_join_directories p450_ld_library_entries
    )"
    p450_require_unshadowed_livox \
      "$p450_final_gazebo_plugin_path" GAZEBO_PLUGIN_PATH
    p450_require_unshadowed_livox \
      "$p450_final_ld_library_path" LD_LIBRARY_PATH

    export P450_PX4_ROOT="$p450_px4_root"
    if [[ -n "$p450_gazebo_display" ]]; then
      export DISPLAY="$p450_gazebo_display"
      export XAUTHORITY="$p450_gazebo_xauthority"
    fi
    export ROS_PACKAGE_PATH="$p450_px4_root:$p450_px4_root/Tools/sitl_gazebo${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
    export GAZEBO_MODEL_PATH="$p450_assets_models:$p450_prometheus_share/gazebo_models/uav_models:$p450_prometheus_share/gazebo_models/sensor_models:$p450_prometheus_share/gazebo_models/scene_models:$p450_prometheus_share/gazebo_models/r200_models:$p450_prometheus_share/gazebo_models/texture:$p450_external_models"
    export GAZEBO_PLUGIN_PATH="$p450_final_gazebo_plugin_path"
    export LD_LIBRARY_PATH="$p450_final_ld_library_path"

    "$p450_repo_root/scripts/validate_p450_current_environment.bash" \
      "$p450_repo_root" "$p450_px4_root"

    exec "$@"
  ' p450-runtime "$p450_repo_root" "$p450_px4_root" \
    "$p450_gazebo_display" "$p450_gazebo_xauthority" "$@"
