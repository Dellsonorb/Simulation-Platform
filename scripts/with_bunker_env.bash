#!/bin/bash

set -euo pipefail

bunker_fail() {
  echo "with-bunker-env: $2" >&2
  exit "$1"
}

bunker_wrapper_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")" || \
  bunker_fail 65 "cannot resolve wrapper path"
bunker_script_dir="${bunker_wrapper_path%/*}"
bunker_repo_root="${bunker_script_dir%/*}"
bunker_run_root="$bunker_repo_root/logs/bunker_standalone"
bunker_install_root="$bunker_repo_root/install/p450-clean"

bunker_validate_run_dir() {
  if [[ $# -ne 1 || "$1" != /* || ! -d "$1" || -L "$1" ]]; then
    return 1
  fi
  local candidate="$1"
  local canonical
  canonical="$(/usr/bin/realpath -e -- "$candidate")" || return 1
  [[ "$canonical" == "$candidate" ]] || return 1
  local canonical_root
  canonical_root="$(/usr/bin/realpath -e -- "$bunker_run_root")" || return 1
  [[ "$canonical_root" == "$bunker_run_root" ]] || return 1
  case "$canonical" in
    "$canonical_root"/*) ;;
    *) return 1 ;;
  esac
  [[ "$(/usr/bin/stat -c '%a' -- "$canonical")" == "700" ]] || return 1
  [[ -r "$canonical" && -w "$canonical" && -x "$canonical" ]] || return 1
  local name
  for name in tmp ros-home ros-log gazebo-log ign-fuel-cache \
      xdg-cache xdg-config xdg-data; do
    [[ ! -e "$canonical/$name" && ! -L "$canonical/$name" ]] || return 1
  done
  printf '%s\n' "$canonical"
}

bunker_known_path_label() {
  case "$1" in
    ROS_PACKAGE_PATH|CMAKE_PREFIX_PATH|PYTHONPATH|LD_LIBRARY_PATH|\
    GAZEBO_PLUGIN_PATH|GAZEBO_MODEL_PATH|GAZEBO_RESOURCE_PATH|\
    PKG_CONFIG_PATH|OGRE_RESOURCE_PATH) return 0 ;;
    *) return 1 ;;
  esac
}

bunker_canonicalize_path_list() {
  if [[ $# -ne 4 ]]; then
    return 1
  fi
  local label="$1"
  local value="$2"
  local run_dir="$3"
  local duplicate_policy="$4"
  bunker_known_path_label "$label" || return 1
  [[ -n "$value" && "$value" != :* && "$value" != *: && \
     "$value" != *::* ]] || return 1
  [[ "$duplicate_policy" == "reject" || \
     "$duplicate_policy" == "dedupe" ]] || return 1
  local canonical_run
  canonical_run="$(bunker_validate_run_dir "$run_dir")" || return 1
  local canonical_install="$bunker_install_root"
  if [[ -e "$canonical_install" ]]; then
    canonical_install="$(/usr/bin/realpath -e -- "$canonical_install")" || \
      return 1
  fi
  local -a components=()
  IFS=: read -r -a components <<< "$value"
  [[ ${#components[@]} -gt 0 ]] || return 1
  local -A seen=()
  local -a output=()
  local component canonical
  for component in "${components[@]}"; do
    [[ -n "$component" && "$component" == /* && \
       -d "$component" && ! -L "$component" && \
       -r "$component" && -x "$component" ]] || return 1
    canonical="$(/usr/bin/realpath -e -- "$component")" || return 1
    [[ "$canonical" == "$component" ]] || return 1
    if [[ "$canonical" =~ (^|/)(src|source|devel|build)(/|$) ]]; then
      return 1
    fi
    case "$canonical" in
      /opt|/opt/*|/usr|/usr/*|"$canonical_install"|"$canonical_install"/*|\
      "$canonical_run"|"$canonical_run"/*) ;;
      *) return 1 ;;
    esac
    if [[ -n "${seen[$canonical]+present}" ]]; then
      [[ "$duplicate_policy" == "dedupe" ]] || return 1
      continue
    fi
    seen["$canonical"]=1
    output+=("$canonical")
  done
  local joined=""
  for component in "${output[@]}"; do
    if [[ -z "$joined" ]]; then
      joined="$component"
    else
      joined="$joined:$component"
    fi
  done
  [[ -n "$joined" ]] || return 1
  printf '%s\n' "$joined"
}

bunker_validate_plugin_candidates() {
  if [[ $# -ne 2 ]]; then
    return 1
  fi
  local gazebo_list="$1"
  local library_list="$2"
  [[ -n "$gazebo_list" && -n "$library_list" && \
     "$gazebo_list" != :* && "$gazebo_list" != *: && \
     "$gazebo_list" != *::* && "$library_list" != :* && \
     "$library_list" != *: && "$library_list" != *::* ]] || return 1
  local -a raw=()
  local -a first=()
  local -a second=()
  IFS=: read -r -a first <<< "$gazebo_list"
  IFS=: read -r -a second <<< "$library_list"
  raw+=("${first[@]}" "${second[@]}")
  local -A seen=()
  local -a directories=()
  local directory canonical
  for directory in "${raw[@]}"; do
    [[ -n "$directory" && "$directory" == /* && \
       -d "$directory" && ! -L "$directory" && \
       -r "$directory" && -x "$directory" ]] || return 1
    canonical="$(/usr/bin/realpath -e -- "$directory")" || return 1
    [[ "$canonical" == "$directory" ]] || return 1
    if [[ -z "${seen[$canonical]+present}" ]]; then
      seen["$canonical"]=1
      directories+=("$canonical")
    fi
  done
  local -a names=(
    libgazebo_ros_planar_move.so
    libgazebo_ros_laser.so
    libRayPlugin.so
  )
  local -a expected=(
    /opt/ros/noetic/lib/libgazebo_ros_planar_move.so
    /opt/ros/noetic/lib/libgazebo_ros_laser.so
    /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so
  )
  local index candidate resolved
  local -a matches=()
  for index in 0 1 2; do
    matches=()
    for directory in "${directories[@]}"; do
      candidate="$directory/${names[$index]}"
      if [[ -e "$candidate" || -L "$candidate" ]]; then
        [[ -f "$candidate" && ! -L "$candidate" && -r "$candidate" ]] || \
          return 1
        resolved="$(/usr/bin/realpath -e -- "$candidate")" || return 1
        [[ "$resolved" == "$candidate" ]] || return 1
        matches+=("$resolved")
      fi
    done
    [[ ${#matches[@]} -eq 1 && \
       "${matches[0]}" == "${expected[$index]}" ]] || return 1
    printf '%s\n' "${matches[0]}"
  done
}

case "${1:-}" in
  --test-run-dir)
    [[ $# -eq 2 ]] || bunker_fail 64 "--test-run-dir requires PATH"
    bunker_validate_run_dir "$2" || bunker_fail 65 "invalid run directory"
    exit 0
    ;;
  --test-path-list)
    [[ $# -eq 4 ]] || bunker_fail 64 \
      "--test-path-list requires LABEL VALUE RUN_DIR"
    bunker_canonicalize_path_list "$2" "$3" "$4" reject || \
      bunker_fail 65 "invalid $2 path list"
    exit 0
    ;;
  --test-fixed-environment)
    [[ $# -eq 7 ]] || bunker_fail 64 \
      "--test-fixed-environment requires six values"
    [[ "$2" == "/opt/ros/noetic/bin:/usr/bin:/bin" && \
       "$3" == "C.UTF-8" && "$4" == "C.UTF-8" && \
       "$5" == "/opt/ros/noetic/etc/ros" && \
       "$6" == "/opt/ros/noetic/share/ros" && -z "$7" ]] || \
      bunker_fail 65 "fixed environment differs"
    exit 0
    ;;
  --test-plugin-candidates)
    [[ $# -eq 3 ]] || bunker_fail 64 \
      "--test-plugin-candidates requires two path lists"
    bunker_validate_plugin_candidates "$2" "$3" || \
      bunker_fail 65 "plugin candidates differ"
    exit 0
    ;;
esac

if [[ $# -lt 4 || "$1" != "--run-dir" || "$3" != "--" ]]; then
  bunker_fail 64 \
    "expected --run-dir RUN_DIR -- COMMAND [ARGUMENT ...]"
fi
bunker_run_dir="$2"
shift 3
[[ $# -gt 0 && -n "$1" ]] || bunker_fail 64 "command is required"
bunker_command=("$@")
bunker_run_dir="$(bunker_validate_run_dir "$bunker_run_dir")" || \
  bunker_fail 65 "invalid run directory"

bunker_login_uid="$(/usr/bin/id -u)"
bunker_login_name="$(/usr/bin/id -un)"
bunker_passwd_record="$(/usr/bin/getent passwd "$bunker_login_uid")" || \
  bunker_fail 65 "cannot resolve login identity"
IFS=: read -r bunker_pw_name bunker_pw_password bunker_pw_uid bunker_pw_gid \
  bunker_pw_gecos bunker_login_home bunker_pw_shell <<< "$bunker_passwd_record"
if [[ -z "$bunker_login_home" || "$bunker_login_home" != /* || \
      "$bunker_pw_name" != "$bunker_login_name" ]]; then
  bunker_fail 65 "login identity is not canonical"
fi

umask 077
bunker_tmp="$bunker_run_dir/tmp"
bunker_ros_home="$bunker_run_dir/ros-home"
bunker_ros_log="$bunker_run_dir/ros-log"
bunker_gazebo_log="$bunker_run_dir/gazebo-log"
bunker_ign_cache="$bunker_run_dir/ign-fuel-cache"
bunker_xdg_cache="$bunker_run_dir/xdg-cache"
bunker_xdg_config="$bunker_run_dir/xdg-config"
bunker_xdg_data="$bunker_run_dir/xdg-data"
/usr/bin/install -d -m 700 -- \
  "$bunker_tmp" "$bunker_ros_home" "$bunker_ros_log" \
  "$bunker_gazebo_log" "$bunker_ign_cache" "$bunker_xdg_cache" \
  "$bunker_xdg_config" "$bunker_xdg_data"

exec /usr/bin/env -i \
  HOME="$bunker_login_home" \
  USER="$bunker_login_name" \
  LOGNAME="$bunker_login_name" \
  SHELL=/bin/bash \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH=/opt/ros/noetic/bin:/usr/bin:/bin \
  BUNKER_REPO_ROOT="$bunker_repo_root" \
  BUNKER_RUN_DIR="$bunker_run_dir" \
  TMPDIR="$bunker_tmp" \
  ROS_HOME="$bunker_ros_home" \
  ROS_LOG_DIR="$bunker_ros_log" \
  GAZEBO_LOG_PATH="$bunker_gazebo_log" \
  IGN_FUEL_CACHE_PATH="$bunker_ign_cache" \
  XDG_CACHE_HOME="$bunker_xdg_cache" \
  XDG_CONFIG_HOME="$bunker_xdg_config" \
  XDG_DATA_HOME="$bunker_xdg_data" \
  /bin/bash --noprofile --norc -c '
set -eo pipefail
umask 077
source /opt/ros/noetic/setup.bash
source "$BUNKER_REPO_ROOT/install/p450-clean/setup.bash"

GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
set -u
source /usr/share/gazebo/setup.sh
GAZEBO_MODEL_DATABASE_URI=""
ROSLISP_PACKAGE_DIRECTORIES=""
export GAZEBO_MODEL_DATABASE_URI ROSLISP_PACKAGE_DIRECTORIES

state_names=(
  TMPDIR ROS_HOME ROS_LOG_DIR GAZEBO_LOG_PATH IGN_FUEL_CACHE_PATH
  XDG_CACHE_HOME XDG_CONFIG_HOME XDG_DATA_HOME
)
state_paths=(
  "$BUNKER_RUN_DIR/tmp" "$BUNKER_RUN_DIR/ros-home"
  "$BUNKER_RUN_DIR/ros-log" "$BUNKER_RUN_DIR/gazebo-log"
  "$BUNKER_RUN_DIR/ign-fuel-cache" "$BUNKER_RUN_DIR/xdg-cache"
  "$BUNKER_RUN_DIR/xdg-config" "$BUNKER_RUN_DIR/xdg-data"
)
for index in 0 1 2 3 4 5 6 7; do
  variable="${state_names[$index]}"
  expected="${state_paths[$index]}"
  actual="${!variable}"
  [[ "$actual" == "$expected" && -d "$actual" && ! -L "$actual" ]]
  [[ "$(/usr/bin/realpath -e -- "$actual")" == "$actual" ]]
  [[ "$(/usr/bin/stat -c "%a" -- "$actual")" == "700" ]]
  case "$actual" in "$BUNKER_RUN_DIR"/*) ;; *) exit 65 ;; esac
done

canonicalize_path_list() {
  local variable="$1"
  local value="${!variable}"
  [[ -n "$value" ]]
  local -a components=()
  IFS=: read -r -a components <<< "$value"
  local -A seen=()
  local -a output=()
  local component canonical
  for component in "${components[@]}"; do
    [[ -n "$component" ]] || continue
    [[ "$component" == /* && -d "$component" && ! -L "$component" && \
       -r "$component" && -x "$component" ]]
    canonical="$(/usr/bin/realpath -e -- "$component")"
    [[ "$canonical" == "$component" ]]
    [[ ! "$canonical" =~ (^|/)(src|source|devel|build)(/|$) ]]
    case "$canonical" in
      /opt|/opt/*|/usr|/usr/*|\
      "$BUNKER_REPO_ROOT/install/p450-clean"|\
      "$BUNKER_REPO_ROOT/install/p450-clean"/*|\
      "$BUNKER_RUN_DIR"|"$BUNKER_RUN_DIR"/*) ;;
      *) return 65 ;;
    esac
    if [[ -z "${seen[$canonical]+present}" ]]; then
      seen["$canonical"]=1
      output+=("$canonical")
    fi
  done
  local joined=""
  for component in "${output[@]}"; do
    [[ -z "$joined" ]] && joined="$component" || joined="$joined:$component"
  done
  [[ -n "$joined" ]]
  printf -v "$variable" "%s" "$joined"
  export "$variable"
}

for variable in ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH \
    LD_LIBRARY_PATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH \
    GAZEBO_RESOURCE_PATH PKG_CONFIG_PATH OGRE_RESOURCE_PATH; do
  canonicalize_path_list "$variable"
done

install="$BUNKER_REPO_ROOT/install/p450-clean"
[[ "$ROS_PACKAGE_PATH" == "$install/share:/opt/ros/noetic/share" ]]
[[ "$CMAKE_PREFIX_PATH" == "$install:/opt/ros/noetic" ]]
[[ "$PYTHONPATH" == "$install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages" ]]
[[ "$LD_LIBRARY_PATH" == "$install/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" ]]
[[ "$GAZEBO_PLUGIN_PATH" == "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib" ]]
[[ "$GAZEBO_MODEL_PATH" == "/usr/share/gazebo-11/models" ]]
[[ "$GAZEBO_RESOURCE_PATH" == "/usr/share/gazebo-11" ]]
[[ "$PKG_CONFIG_PATH" == "$install/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig" ]]
[[ "$OGRE_RESOURCE_PATH" == "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0" ]]
[[ "$ROS_ETC_DIR" == "/opt/ros/noetic/etc/ros" ]]
[[ "$ROS_ROOT" == "/opt/ros/noetic/share/ros" ]]
[[ -z "$ROSLISP_PACKAGE_DIRECTORIES" ]]
[[ "$PATH" == "/opt/ros/noetic/bin:/usr/bin:/bin" ]]
[[ "$LANG" == "C.UTF-8" && "$LC_ALL" == "C.UTF-8" ]]
"$BUNKER_REPO_ROOT/scripts/with_bunker_env.bash" \
  --test-plugin-candidates "$GAZEBO_PLUGIN_PATH" "$LD_LIBRARY_PATH" \
  >/dev/null

command=("$@")
[[ ${#command[@]} -gt 0 && -n "${command[0]}" ]]
exec "${command[@]}"
' bunker-runtime "${bunker_command[@]}"
