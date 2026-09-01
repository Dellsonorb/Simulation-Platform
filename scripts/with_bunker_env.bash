#!/bin/bash
# shellcheck disable=SC2016
# The single-quoted block is an intentional clean-shell program.

set -euo pipefail

usage() {
  cat <<'EOF'
usage: scripts/with_bunker_env.bash --run-dir DIR -- COMMAND [ARG ...]

Run a command against the installed BUNKER workspace while keeping ROS,
Gazebo, and cache output inside DIR.
EOF
}

if [[ "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -lt 4 || "$1" != "--run-dir" || "$3" != "--" ]]; then
  usage >&2
  exit 64
fi

run_dir="$2"
shift 3
[[ $# -gt 0 ]] || { usage >&2; exit 64; }

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
install_root="$repo_root/install/p450-clean"
[[ -f "$install_root/setup.bash" ]] || {
  echo "with-bunker-env: build install/p450-clean first" >&2
  exit 65
}

/usr/bin/mkdir -p -- "$run_dir"
run_dir="$(/usr/bin/realpath -e -- "$run_dir")"
umask 077
for name in home tmp ros-home ros-log gazebo-log ign-cache xdg-cache xdg-config; do
  /usr/bin/mkdir -p -- "$run_dir/$name"
done

login_name="$(/usr/bin/id -un)"
ros_master_uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
gazebo_master_uri="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"

exec /usr/bin/env -i \
  HOME="$run_dir/home" \
  USER="$login_name" LOGNAME="$login_name" SHELL=/bin/bash \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  PATH=/opt/ros/noetic/bin:/usr/bin:/bin \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
  ROS_MASTER_URI="$ros_master_uri" \
  GAZEBO_MASTER_URI="$gazebo_master_uri" \
  ROS_HOME="$run_dir/ros-home" ROS_LOG_DIR="$run_dir/ros-log" \
  GAZEBO_LOG_PATH="$run_dir/gazebo-log" \
  IGN_FUEL_CACHE_PATH="$run_dir/ign-cache" \
  XDG_CACHE_HOME="$run_dir/xdg-cache" \
  XDG_CONFIG_HOME="$run_dir/xdg-config" \
  TMPDIR="$run_dir/tmp" \
  BUNKER_REPO_ROOT="$repo_root" BUNKER_RUN_DIR="$run_dir" \
  /bin/bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/noetic/setup.bash
source "$BUNKER_REPO_ROOT/install/p450-clean/setup.bash"
GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
GAZEBO_PLUGIN_PATH="$BUNKER_REPO_ROOT/install/p450-clean/lib:/opt/ros/noetic/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_DATABASE_URI=""
exec "$@"
' bunker-env "$@"
