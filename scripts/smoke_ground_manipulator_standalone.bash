#!/bin/bash
# shellcheck disable=SC2016

set -euo pipefail

usage() {
  cat <<'EOF'
Ground manipulator standalone smoke

usage: scripts/smoke_ground_manipulator_standalone.bash [OPTIONS]

Options:
  --run-root DIR       Store one run directory below DIR.
  --gui true|false     Start Gazebo client (default: false).
  --ros-port PORT      ROS master port (default: 11771).
  --gazebo-port PORT   Gazebo master port (default: 11772).
  --help               Show this help without starting ROS or Gazebo.
EOF
}

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
run_root="$repo_root/logs/ground_manipulator_standalone"
gui=false
ros_port=11771
gazebo_port=11772

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --run-root) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; run_root="$2"; shift 2 ;;
    --gui) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gui="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; ros_port="$2"; shift 2 ;;
    --gazebo-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gazebo_port="$2"; shift 2 ;;
    *) echo "smoke-ground-manipulator: unknown option $1" >&2; usage >&2; exit 64 ;;
  esac
done

[[ "$gui" == true || "$gui" == false ]] || {
  echo "smoke-ground-manipulator: --gui must be true or false" >&2
  exit 64
}
[[ "$ros_port" =~ ^[0-9]+$ && "$gazebo_port" =~ ^[0-9]+$ ]] || {
  echo "smoke-ground-manipulator: ports must be integers" >&2
  exit 64
}

/usr/bin/mkdir -p -- "$run_root"
run_dir="$(/usr/bin/mktemp -d -p "$run_root" "$(/bin/date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
echo "Ground manipulator smoke run: $run_dir"

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
    for _ in {1..150}; do
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
    echo "smoke-ground-manipulator: roslaunch exited with status $launch_status" >&2
    status=1
  fi
  exit "$status"
}
trap cleanup EXIT
trap "exit 130" INT TERM

setsid roslaunch bunker_aubo_moveit_config moveit_planning_execution.launch \
  start_gazebo:=true gui:="$gui" rviz:=false \
  >"$run_dir/runtime.log" 2>&1 &
launch_pid=$!

sim_time_ready=false
for _ in {1..300}; do
  if [[ "$(rosparam get /use_sim_time 2>/dev/null || true)" == true ]]; then
    sim_time_ready=true
    break
  fi
  if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
    echo "smoke-ground-manipulator: launcher exited before ROS was ready" >&2
    exit 1
  fi
  /bin/sleep 0.1
done
if [[ "$sim_time_ready" != true ]]; then
  echo "smoke-ground-manipulator: /use_sim_time was not ready" >&2
  exit 1
fi
/usr/bin/timeout --signal=TERM --kill-after=2s 30s \
  rostopic echo -n 1 /clock >/dev/null

/usr/bin/timeout --signal=TERM --kill-after=5s 180s \
  /usr/bin/python3 -B "$repo_root/scripts/check_ground_manipulator_runtime.py" \
    --summary "$run_dir/summary.json" --timeout 30

if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
  echo "smoke-ground-manipulator: launcher exited during checks" >&2
  exit 1
fi
' ground-manipulator-smoke "$repo_root" "$run_dir" "$gui"

echo "PASS: Ground manipulator standalone smoke"
echo "Summary: $run_dir/summary.json"
