#!/bin/bash

set -euo pipefail

usage() {
  cat <<'EOF'
Natural Air-Ground Pick Demo smoke

usage: P450_PX4_ROOT=DIR scripts/smoke_air_ground_pick_demo.bash [OPTIONS]

Options:
  --run-root DIR       Store one run directory below DIR.
  --gui true|false     Start Gazebo client (default: false).
  --ros-port PORT      ROS master port (default: 11771).
  --gazebo-port PORT   Gazebo master port (default: 11772).
  --timeout SECONDS    Bound the full demo (default: 360).
  --help               Show this help without starting ROS or Gazebo.
EOF
}

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
run_root="$repo_root/logs/air_ground_pick_demo"
gui=false
ros_port=11771
gazebo_port=11772
timeout_seconds=360

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --run-root) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; run_root="$2"; shift 2 ;;
    --gui) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gui="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; ros_port="$2"; shift 2 ;;
    --gazebo-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gazebo_port="$2"; shift 2 ;;
    --timeout) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; timeout_seconds="$2"; shift 2 ;;
    *) echo "smoke-air-ground-pick: unknown option $1" >&2; usage >&2; exit 64 ;;
  esac
done

[[ "$gui" == true || "$gui" == false ]] || {
  echo "smoke-air-ground-pick: --gui must be true or false" >&2
  exit 64
}
[[ "$ros_port" =~ ^[0-9]+$ && "$gazebo_port" =~ ^[0-9]+$ &&
   "$timeout_seconds" =~ ^[1-9][0-9]*$ ]] || {
  echo "smoke-air-ground-pick: ports and timeout must be positive integers" >&2
  exit 64
}

if [[ "${AIR_GROUND_PICK_SMOKE_INNER:-0}" != 1 ]]; then
  [[ -n "${P450_PX4_ROOT:-}" ]] || {
    echo "smoke-air-ground-pick: P450_PX4_ROOT is required" >&2
    exit 64
  }
  export P450_GAZEBO_DISPLAY="${P450_GAZEBO_DISPLAY:-${DISPLAY:-}}"
  export P450_GAZEBO_XAUTHORITY="${P450_GAZEBO_XAUTHORITY:-${XAUTHORITY:-}}"
  exec "$repo_root/scripts/with_p450_env.bash" /usr/bin/env \
    AIR_GROUND_PICK_SMOKE_INNER=1 \
    ROS_MASTER_URI="http://127.0.0.1:$ros_port" \
    GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port" \
    "$script_path" --run-root "$run_root" --gui "$gui" \
      --ros-port "$ros_port" --gazebo-port "$gazebo_port" \
      --timeout "$timeout_seconds"
fi

/usr/bin/mkdir -p -- "$run_root"
run_dir="$(/usr/bin/mktemp -d -p "$run_root" "$(/bin/date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
px4_workdir="sitl_air_ground_pick_$(/bin/date -u +%Y%m%dT%H%M%SZ)_$$"
echo "Air-Ground Pick smoke run: $run_dir"

launch_pid=""
launch_pgid=""
checks_complete=false

launch_group_running() {
  [[ -n "$launch_pgid" ]] || return 1
  /bin/kill -0 -- "-$launch_pgid" 2>/dev/null
}

wait_for_launch_group() {
  attempts="$1"
  for ((index = 0; index < attempts; index++)); do
    launch_group_running || return 0
    /bin/sleep 0.1
  done
  return 1
}

cleanup() {
  status=$?
  trap - EXIT INT TERM
  if launch_group_running; then
    /bin/kill -INT -- "-$launch_pgid" 2>/dev/null || true
    wait_for_launch_group 200 || true
  fi
  if launch_group_running; then
    /bin/kill -TERM -- "-$launch_pgid" 2>/dev/null || true
    wait_for_launch_group 100 || true
  fi
  if launch_group_running; then
    /bin/kill -KILL -- "-$launch_pgid" 2>/dev/null || true
    wait_for_launch_group 50 || true
  fi
  if launch_group_running; then
    echo "smoke-air-ground-pick: launch process group did not exit" >&2
    status=1
  fi
  if [[ -n "$launch_pid" ]] && ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ "$status" -eq 0 && "$checks_complete" == true ]]; then
    echo "PASS: natural Air-Ground Pick Demo smoke"
    echo "Summary: $run_dir/summary.json"
  fi
  exit "$status"
}
trap cleanup EXIT
trap "exit 130" INT TERM

setsid roslaunch air_ground_pick_demo air_ground_pick_demo.launch \
  gui:="$gui" enable_mid360:=true px4_workdir:="$px4_workdir" \
  >"$run_dir/runtime.log" 2>&1 &
launch_pid=$!
observed_pgid="$(/bin/ps -o pgid= -p "$launch_pid" | /usr/bin/tr -d '[:space:]')"
if [[ ! "$observed_pgid" =~ ^[1-9][0-9]*$ || "$observed_pgid" != "$launch_pid" ]]; then
  echo "smoke-air-ground-pick: setsid did not create the expected process group" >&2
  /bin/kill -TERM "$launch_pid" 2>/dev/null || true
  for _ in {1..100}; do
    /bin/kill -0 "$launch_pid" 2>/dev/null || break
    /bin/sleep 0.1
  done
  /bin/kill -KILL "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  launch_pid=""
  exit 1
fi
launch_pgid="$observed_pgid"

clock_ready=false
for _ in {1..600}; do
  if rostopic list 2>/dev/null | /bin/grep -qx /clock; then
    clock_ready=true
    break
  fi
  if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
    echo "smoke-air-ground-pick: launcher exited before /clock" >&2
    exit 1
  fi
  /bin/sleep 0.1
done
if [[ "$clock_ready" != true ]]; then
  echo "smoke-air-ground-pick: /clock was not ready" >&2
  exit 1
fi

checker_timeout=$((timeout_seconds + 10))
/usr/bin/timeout --signal=TERM --kill-after=5s "${checker_timeout}s" \
  /usr/bin/python3 -B "$repo_root/scripts/check_air_ground_pick_demo.py" \
    --summary "$run_dir/summary.json" --timeout "$timeout_seconds"

launch_exited=false
for _ in {1..300}; do
  if ! launch_group_running; then
    launch_exited=true
    break
  fi
  /bin/sleep 0.1
done
if [[ "$launch_exited" != true ]]; then
  echo "smoke-air-ground-pick: launcher did not stop after LIFT" >&2
  exit 1
fi
if wait "$launch_pid"; then
  launch_status=0
else
  launch_status=$?
  echo "smoke-air-ground-pick: roslaunch exited with status $launch_status" >&2
  exit 1
fi
/usr/bin/python3 -B "$repo_root/scripts/check_air_ground_pick_demo.py" \
  --finalize-summary "$run_dir/summary.json"
launch_pid=""
launch_pgid=""
checks_complete=true
