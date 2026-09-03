#!/bin/bash

set -euo pipefail

usage() {
  /bin/cat <<'EOF'
Deterministic RM4D -> Air-Ground Pick smoke

usage: P450_PX4_ROOT=DIR RM4D_ROOT=DIR RM4D_PYTHON=FILE RM4D_MAP=FILE \
  scripts/smoke_rm4d_air_ground_pick_demo.bash [OPTIONS]

Options:
  --run-root DIR       Store one run directory below DIR.
  --gui true|false     Start Gazebo client (default: false).
  --rviz true|false    Start the RM4D RViz view (default: false).
  --ros-port PORT      ROS master port (default: 11871).
  --gazebo-port PORT   Gazebo master port (default: 11872).
  --timeout SECONDS    Bound the full demo (default: 480).
  --help               Show this help without starting the platform.
EOF
}

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
run_root="$repo_root/logs/rm4d_air_ground_pick_demo"
gui=false
rviz=false
ros_port=11871
gazebo_port=11872
timeout_seconds=480

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --run-root) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; run_root="$2"; shift 2 ;;
    --gui) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gui="$2"; shift 2 ;;
    --rviz) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; rviz="$2"; shift 2 ;;
    --ros-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; ros_port="$2"; shift 2 ;;
    --gazebo-port) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; gazebo_port="$2"; shift 2 ;;
    --timeout) [[ $# -ge 2 ]] || { usage >&2; exit 64; }; timeout_seconds="$2"; shift 2 ;;
    *) echo "smoke-rm4d-pick: unknown option $1" >&2; usage >&2; exit 64 ;;
  esac
done

[[ "$gui" == true || "$gui" == false ]] || {
  echo "smoke-rm4d-pick: --gui must be true or false" >&2
  exit 64
}
[[ "$rviz" == true || "$rviz" == false ]] || {
  echo "smoke-rm4d-pick: --rviz must be true or false" >&2
  exit 64
}
[[ "$ros_port" =~ ^[0-9]+$ && "$gazebo_port" =~ ^[0-9]+$ &&
   "$timeout_seconds" =~ ^[1-9][0-9]*$ ]] || {
  echo "smoke-rm4d-pick: ports and timeout must be positive integers" >&2
  exit 64
}

if [[ "${RM4D_PICK_SMOKE_INNER:-0}" != 1 ]]; then
  [[ -n "${P450_PX4_ROOT:-}" ]] || {
    echo "smoke-rm4d-pick: P450_PX4_ROOT is required" >&2
    exit 64
  }
  [[ -d "${RM4D_ROOT:-}" ]] || {
    echo "smoke-rm4d-pick: RM4D_ROOT must be a directory" >&2
    exit 64
  }
  [[ -x "${RM4D_PYTHON:-}" ]] || {
    echo "smoke-rm4d-pick: RM4D_PYTHON must be executable" >&2
    exit 64
  }
  [[ -f "${RM4D_MAP:-}" ]] || {
    echo "smoke-rm4d-pick: RM4D_MAP must be a file" >&2
    exit 64
  }
  export P450_GAZEBO_DISPLAY="${P450_GAZEBO_DISPLAY:-${DISPLAY:-}}"
  export P450_GAZEBO_XAUTHORITY="${P450_GAZEBO_XAUTHORITY:-${XAUTHORITY:-}}"
  exec "$repo_root/scripts/with_p450_env.bash" /usr/bin/env \
    RM4D_PICK_SMOKE_INNER=1 \
    RM4D_ROOT="$RM4D_ROOT" \
    RM4D_PYTHON="$RM4D_PYTHON" \
    RM4D_MAP="$RM4D_MAP" \
    RM4D_CONFIG="${RM4D_CONFIG:-}" \
    ROS_MASTER_URI="http://127.0.0.1:$ros_port" \
    GAZEBO_MASTER_URI="http://127.0.0.1:$gazebo_port" \
    "$script_path" --run-root "$run_root" --gui "$gui" --rviz "$rviz" \
      --ros-port "$ros_port" --gazebo-port "$gazebo_port" \
      --timeout "$timeout_seconds"
fi

/usr/bin/mkdir -p -- "$run_root"
run_dir="$(/usr/bin/mktemp -d -p "$run_root" "$({ /bin/date -u +%Y%m%dT%H%M%SZ; })-XXXXXX")"
px4_workdir="sitl_rm4d_pick_$({ /bin/date -u +%Y%m%dT%H%M%SZ; })_$$"
echo "RM4D Air-Ground Pick smoke run: $run_dir"

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
    echo "smoke-rm4d-pick: launch process group did not exit" >&2
    status=1
  fi
  if [[ -n "$launch_pid" ]] && ! /bin/kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ "$status" -eq 0 && "$checks_complete" == true ]]; then
    echo "PASS: deterministic RM4D-driven Air-Ground Pick"
    echo "Summary: $run_dir/summary.json"
    echo "RM4D_SIM_INTEGRATION_READY"
  fi
  exit "$status"
}
trap cleanup EXIT
trap "exit 130" INT TERM

setsid roslaunch rm4d_sim_integration rm4d_air_ground_pick_demo.launch \
  gui:="$gui" rviz:="$rviz" enable_mid360:=true \
  px4_workdir:="$px4_workdir" >"$run_dir/runtime.log" 2>&1 &
launch_pid=$!
observed_pgid="$(/bin/ps -o pgid= -p "$launch_pid" | /usr/bin/tr -d '[:space:]')"
if [[ ! "$observed_pgid" =~ ^[1-9][0-9]*$ || "$observed_pgid" != "$launch_pid" ]]; then
  echo "smoke-rm4d-pick: setsid did not create a process group" >&2
  /bin/kill -TERM "$launch_pid" 2>/dev/null || true
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
    echo "smoke-rm4d-pick: launcher exited before /clock" >&2
    exit 1
  fi
  /bin/sleep 0.1
done
[[ "$clock_ready" == true ]] || {
  echo "smoke-rm4d-pick: /clock was not ready" >&2
  exit 1
}

checker_timeout=$((timeout_seconds + 10))
/usr/bin/timeout --signal=TERM --kill-after=5s "${checker_timeout}s" \
  /usr/bin/python3 -B "$repo_root/scripts/check_air_ground_pick_demo.py" \
    --summary "$run_dir/summary.json" --timeout "$timeout_seconds"

for _ in {1..300}; do
  launch_group_running || break
  /bin/sleep 0.1
done
launch_group_running && {
  echo "smoke-rm4d-pick: launcher did not stop after LIFT" >&2
  exit 1
}
if ! wait "$launch_pid"; then
  echo "smoke-rm4d-pick: roslaunch did not exit cleanly" >&2
  exit 1
fi

rm4d_match="$(/bin/grep -n -m1 '] RM4D_CANDIDATES ' "$run_dir/runtime.log" || true)"
stopped_match="$(/bin/grep -n -m1 '] GROUND_STOPPED ' "$run_dir/runtime.log" || true)"
lift_match="$(/bin/grep -n -m1 '] LIFT ' "$run_dir/runtime.log" || true)"
rm4d_line="${rm4d_match%%:*}"
stopped_line="${stopped_match%%:*}"
lift_line="${lift_match%%:*}"
if [[ ! "$rm4d_line" =~ ^[1-9][0-9]*$ ||
      ! "$stopped_line" =~ ^[1-9][0-9]*$ ||
      ! "$lift_line" =~ ^[1-9][0-9]*$ ||
      "$rm4d_line" -ge "$stopped_line" || "$stopped_line" -ge "$lift_line" ]]; then
  echo "smoke-rm4d-pick: required state order was not observed" >&2
  exit 1
fi

/usr/bin/python3 -B "$repo_root/scripts/check_air_ground_pick_demo.py" \
  --finalize-summary "$run_dir/summary.json"
launch_pid=""
launch_pgid=""
checks_complete=true
