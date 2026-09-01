#!/bin/bash
# shellcheck disable=SC1004,SC2016
# The single-quoted block is executed by the isolated BUNKER shell.

set -euo pipefail

usage() {
  cat <<'EOF'
BUNKER standalone smoke

usage: scripts/smoke_bunker_standalone.bash [OPTIONS]

Options:
  --run-root DIR       Store one run directory below DIR.
  --gui true|false     Start Gazebo client (default: false).
  --ros-port PORT      ROS master port (default: 11371).
  --gazebo-port PORT   Gazebo master port (default: 11372).
  --help               Show this help without starting ROS or Gazebo.
EOF
}

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
run_root="$repo_root/logs/bunker_standalone"
gui=false
ros_port=11371
gazebo_port=11372

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --run-root) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; run_root="$2"; shift 2 ;;
    --gui) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gui="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; ros_port="$2"; shift 2 ;;
    --gazebo-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gazebo_port="$2"; shift 2 ;;
    *) echo "smoke-bunker-standalone: unknown option $1" >&2; usage >&2; exit 64 ;;
  esac
done

[[ "$gui" == true || "$gui" == false ]] || {
  echo "smoke-bunker-standalone: --gui must be true or false" >&2
  exit 64
}
[[ "$ros_port" =~ ^[0-9]+$ && "$gazebo_port" =~ ^[0-9]+$ ]] || {
  echo "smoke-bunker-standalone: ports must be integers" >&2
  exit 64
}

/usr/bin/mkdir -p -- "$run_root"
run_dir="$(/usr/bin/mktemp -d -p "$run_root" "$(/bin/date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
echo "BUNKER smoke run: $run_dir"

export ROS_MASTER_URI="http://127.0.0.1:$ros_port"
export GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port"

"$repo_root/scripts/with_bunker_env.bash" --run-dir "$run_dir" -- \
  /bin/bash --noprofile --norc -c '
set -euo pipefail
repo_root="$1"
run_dir="$2"
gui="$3"

launch_pid=""
cleanup() {
  status=$?
  launch_status=0
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && /bin/kill -0 "$launch_pid" 2>/dev/null; then
    /bin/kill -INT "$launch_pid" 2>/dev/null || true
    for _ in {1..75}; do
      /bin/kill -0 "$launch_pid" 2>/dev/null || break
      /bin/sleep 0.2
    done
    if /bin/kill -0 "$launch_pid" 2>/dev/null; then
      /bin/kill -TERM "$launch_pid" 2>/dev/null || true
    fi
  fi
  if [[ -n "$launch_pid" ]]; then
    if wait "$launch_pid" 2>/dev/null; then
      launch_status=0
    else
      launch_status=$?
    fi
  fi
  if [[ "$status" -eq 0 && "$launch_status" -ne 0 ]]; then
    echo "smoke-bunker-standalone: roslaunch exited with status $launch_status" >&2
    status=1
  fi
  exit "$status"
}
trap cleanup EXIT
trap "exit 130" INT TERM

setsid roslaunch bunker_sim_runtime bunker_standalone.launch gui:="$gui" \
  >"$run_dir/runtime.log" 2>&1 &
launch_pid=$!

/usr/bin/timeout --signal=TERM --kill-after=5s 120s \
  /usr/bin/python3 -B "$repo_root/scripts/check_bunker_runtime.py" \
  --summary "$run_dir/summary.json"

delete_result="$(rosservice call /gazebo/delete_model "model_name: bunker" 2>&1)" || {
  echo "$delete_result" >&2
  echo "smoke-bunker-standalone: could not unload BUNKER model" >&2
  exit 1
}
[[ "$delete_result" == *"success: True"* ]] || {
  echo "$delete_result" >&2
  echo "smoke-bunker-standalone: Gazebo rejected BUNKER model unload" >&2
  exit 1
}
/bin/sleep 0.5
if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
  echo "smoke-bunker-standalone: launcher exited during model unload" >&2
  exit 1
fi
' bunker-smoke "$repo_root" "$run_dir" "$gui"

echo "PASS: BUNKER standalone smoke"
echo "Summary: $run_dir/summary.json"
