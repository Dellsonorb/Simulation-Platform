#!/bin/bash
# shellcheck disable=SC2016

set -euo pipefail

usage() {
  cat <<'EOF'
Air-ground shared-world smoke

usage: P450_PX4_ROOT=DIR scripts/smoke_air_ground_standalone.bash [OPTIONS]

Options:
  --run-root DIR       Store one run directory below DIR.
  --gui true|false     Start Gazebo client (default: false).
  --ros-port PORT      ROS master port (default: 11671).
  --gazebo-port PORT   Gazebo master port (default: 11672).
  --help               Show this help without starting ROS or Gazebo.
EOF
}

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
run_root="$repo_root/logs/air_ground_standalone"
gui=false
ros_port=11671
gazebo_port=11672

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --run-root) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; run_root="$2"; shift 2 ;;
    --gui) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gui="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; ros_port="$2"; shift 2 ;;
    --gazebo-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gazebo_port="$2"; shift 2 ;;
    *) echo "smoke-air-ground: unknown option $1" >&2; usage >&2; exit 64 ;;
  esac
done

[[ "$gui" == true || "$gui" == false ]] || {
  echo "smoke-air-ground: --gui must be true or false" >&2
  exit 64
}
[[ "$ros_port" =~ ^[0-9]+$ && "$gazebo_port" =~ ^[0-9]+$ ]] || {
  echo "smoke-air-ground: ports must be integers" >&2
  exit 64
}

if [[ "${AIR_GROUND_SMOKE_INNER:-0}" != 1 ]]; then
  [[ -n "${P450_PX4_ROOT:-}" ]] || {
    echo "smoke-air-ground: P450_PX4_ROOT is required" >&2
    exit 64
  }
  export P450_GAZEBO_DISPLAY="${DISPLAY:-}"
  export P450_GAZEBO_XAUTHORITY="${XAUTHORITY:-}"
  exec "$repo_root/scripts/with_p450_env.bash" /usr/bin/env \
    AIR_GROUND_SMOKE_INNER=1 \
    ROS_MASTER_URI="http://127.0.0.1:$ros_port" \
    GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port" \
    "$script_path" --run-root "$run_root" --gui "$gui" \
      --ros-port "$ros_port" --gazebo-port "$gazebo_port"
fi

/usr/bin/mkdir -p -- "$run_root"
run_dir="$(/usr/bin/mktemp -d -p "$run_root" "$(/bin/date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
px4_workdir="sitl_air_ground_$(/bin/date -u +%Y%m%dT%H%M%SZ)_$$"
echo "Air-ground smoke run: $run_dir"

launch_pid=""
checks_complete=false
cleanup() {
  status=$?
  launch_status=0
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && /bin/kill -0 "$launch_pid" 2>/dev/null; then
    /bin/kill -INT "$launch_pid" 2>/dev/null || true
    for _ in {1..300}; do
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
    echo "smoke-air-ground: roslaunch exited with status $launch_status" >&2
    status=1
  fi
  if [[ "$status" -eq 0 && "$checks_complete" == true ]]; then
    echo "PASS: P450 + Ground Robot shared-world smoke"
    echo "Summary: $run_dir/summary.json"
  fi
  exit "$status"
}
trap cleanup EXIT
trap "exit 130" INT TERM

setsid roslaunch sim_platform_bringup air_ground_standalone.launch \
  gui:="$gui" use_sim_time:=true enable_mid360:=true \
  px4_workdir:="$px4_workdir" \
  >"$run_dir/runtime.log" 2>&1 &
launch_pid=$!

sim_time_ready=false
for _ in {1..300}; do
  if [[ "$(rosparam get /use_sim_time 2>/dev/null || true)" == true ]]; then
    sim_time_ready=true
    break
  fi
  if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
    echo "smoke-air-ground: shared launcher exited before ROS was ready" >&2
    exit 1
  fi
  /bin/sleep 0.1
done
if [[ "$sim_time_ready" != true ]]; then
  echo "smoke-air-ground: /use_sim_time was not ready" >&2
  exit 1
fi
/usr/bin/timeout --signal=TERM --kill-after=2s 30s \
  rostopic echo -n 1 /clock >/dev/null

/usr/bin/timeout --signal=TERM --kill-after=5s 180s \
  /usr/bin/python3 -B "$repo_root/scripts/check_air_ground_runtime.py" \
    --summary "$run_dir/summary.json" --timeout 30

if ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
  echo "smoke-air-ground: shared launcher exited during checks" >&2
  exit 1
fi

checks_complete=true
