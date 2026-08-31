#!/bin/bash

set -euo pipefail

if [[ -z "${P450_PX4_ROOT:-}" ]]; then
  echo "P450_PX4_ROOT must be set by scripts/with_p450_env.bash" >&2
  exit 64
fi

if [[ "$P450_PX4_ROOT" != /* ]]; then
  echo "P450_PX4_ROOT must be an absolute canonical directory" >&2
  exit 65
fi

if ! p450_px4_canonical="$(/usr/bin/realpath -e -- "$P450_PX4_ROOT")" || \
   [[ ! -d "$p450_px4_canonical" ]] || \
   [[ "$p450_px4_canonical" != "$P450_PX4_ROOT" ]]; then
  echo "P450_PX4_ROOT must be an absolute canonical directory" >&2
  exit 65
fi

p450_px4_binary="$p450_px4_canonical/build/amovlab_sitl_default/bin/px4"
if [[ ! -f "$p450_px4_binary" || ! -x "$p450_px4_binary" ]]; then
  echo "PX4 SITL binary is missing or not executable: $p450_px4_binary" >&2
  exit 66
fi

if [[ -z "${ROS_HOME:-}" || "$ROS_HOME" != /* ]]; then
  echo "ROS_HOME must name an absolute existing directory" >&2
  exit 67
fi

if ! p450_ros_home_canonical="$(/usr/bin/realpath -e -- "$ROS_HOME")" || \
   [[ ! -d "$p450_ros_home_canonical" ]] || \
   [[ "$p450_ros_home_canonical" == "/" ]]; then
  echo "ROS_HOME must name an absolute existing directory below /" >&2
  exit 67
fi

p450_px4_args=("$@")
p450_workdir_count=0
p450_workdir_has_value=0
p450_workdir_component=""
for ((p450_arg_index = 0;
      p450_arg_index < ${#p450_px4_args[@]};
      p450_arg_index += 1)); do
  if [[ "${p450_px4_args[$p450_arg_index]}" != "-w" ]]; then
    continue
  fi

  ((p450_workdir_count += 1))
  if ((p450_arg_index + 1 < ${#p450_px4_args[@]})); then
    p450_workdir_component="${p450_px4_args[$((p450_arg_index + 1))]}"
    p450_workdir_has_value=1
  fi
done

if ((p450_workdir_count != 1 || p450_workdir_has_value != 1)); then
  echo "PX4 SITL requires exactly one -w WORKDIR argument" >&2
  exit 68
fi

if [[ ! "$p450_workdir_component" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "PX4 workdir must be a safe single path component" >&2
  exit 68
fi

p450_workdir_candidate="$p450_ros_home_canonical/$p450_workdir_component"
if ! p450_workdir_canonical="$(
  /usr/bin/realpath -m -- "$p450_workdir_candidate"
)" || [[ "$p450_workdir_canonical" != "$p450_ros_home_canonical"/* ]]; then
  echo "PX4 workdir must resolve strictly inside ROS_HOME" >&2
  exit 68
fi

cd -- "$p450_ros_home_canonical"
exec "$p450_px4_binary" "${p450_px4_args[@]}"
