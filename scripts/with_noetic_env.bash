#!/bin/bash

set -euo pipefail

if [[ $# -eq 0 ]]; then
  echo "usage: scripts/with_noetic_env.bash COMMAND [ARG ...]" >&2
  exit 64
fi

p450_wrapper_path="$(/usr/bin/realpath -e "${BASH_SOURCE[0]}")"
p450_script_dir="${p450_wrapper_path%/*}"
p450_repo_root="${p450_script_dir%/*}"

p450_login_uid="$(/usr/bin/id -u)"
p450_login_name="$(/usr/bin/id -un)"
p450_passwd_record="$(/usr/bin/getent passwd "$p450_login_uid")"
IFS=: read -r p450_pw_name p450_pw_password p450_pw_uid p450_pw_gid \
  p450_pw_gecos p450_login_home p450_pw_shell <<< "$p450_passwd_record"

if [[ -z "$p450_login_home" || "$p450_login_home" != /* || \
      "$p450_pw_name" != "$p450_login_name" ]]; then
  echo "unable to resolve the canonical login identity" >&2
  exit 65
fi

p450_ros_home="$p450_repo_root/logs/ros"
p450_ros_log_dir="$p450_ros_home/log"
p450_gazebo_log_path="$p450_repo_root/logs/gazebo"
p450_xdg_config_home="$p450_repo_root/logs/xdg/config"
p450_xdg_cache_home="$p450_repo_root/logs/xdg/cache"

/usr/bin/mkdir -p \
  "$p450_ros_home" \
  "$p450_ros_log_dir" \
  "$p450_gazebo_log_path" \
  "$p450_xdg_config_home" \
  "$p450_xdg_cache_home"

exec /usr/bin/env -i \
  HOME="$p450_login_home" \
  USER="$p450_login_name" \
  LOGNAME="$p450_login_name" \
  SHELL=/bin/bash \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH=/opt/ros/noetic/bin:/usr/bin:/bin:/usr/sbin:/sbin \
  ROS_HOME="$p450_ros_home" \
  ROS_LOG_DIR="$p450_ros_log_dir" \
  GAZEBO_LOG_PATH="$p450_gazebo_log_path" \
  XDG_CONFIG_HOME="$p450_xdg_config_home" \
  XDG_CACHE_HOME="$p450_xdg_cache_home" \
  /bin/bash --noprofile --norc -c \
  'p450_command=("$@"); set --; source /opt/ros/noetic/setup.bash || exit "$?"; exec "${p450_command[@]}"' \
  p450-noetic "$@"
