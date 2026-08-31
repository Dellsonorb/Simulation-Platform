#!/bin/bash

set -euo pipefail

p450_die() {
  echo "P450 standalone smoke: $*" >&2
  exit 1
}

p450_snapshot_has_live_descendant() {
  local p450_root_pid="$1"
  local p450_needle="$2"
  if [[ ! "$p450_root_pid" =~ ^[1-9][0-9]*$ ]] || \
     [[ "$p450_root_pid" == "1" ]] || [[ -z "$p450_needle" ]]; then
    return 64
  fi

  /usr/bin/awk -v root_pid="$p450_root_pid" -v needle="$p450_needle" '
    NF >= 5 && $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/ {
      pid = $1
      parent[pid] = $2
      state[pid] = $4
      command[pid] = ""
      for (field = 5; field <= NF; ++field)
        command[pid] = command[pid] (field == 5 ? "" : OFS) $field
    }
    END {
      for (pid in parent) {
        if (pid == root_pid || state[pid] ~ /^Z/ ||
            index(command[pid], needle) == 0)
          continue

        generation += 1
        cursor = pid
        while ((cursor in parent) && seen[cursor] != generation) {
          seen[cursor] = generation
          cursor = parent[cursor]
          if (cursor == root_pid) {
            found = 1
            break
          }
        }
      }
      exit(found ? 0 : 1)
    }
  '
}

if [[ "${1:-}" == "--test-process-tree" ]]; then
  [[ $# -eq 3 ]] || exit 64
  if p450_snapshot_has_live_descendant "$2" "$3"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_launch_log_is_clean() {
  local p450_candidate_log="$1"
  local p450_fatal_pattern
  local p450_startup_success_count
  p450_fatal_pattern='Segmentation fault|Assertion .* failed|Aborted( \(core dumped\))?|Reboot PX4!|process has died|Startup script returned with return value:'

  if [[ ! -f "$p450_candidate_log" || ! -r "$p450_candidate_log" ]]; then
    echo "launch log must name a readable regular file" >&2
    return 66
  fi
  if /usr/bin/grep -Eq -- "$p450_fatal_pattern" "$p450_candidate_log"; then
    /usr/bin/grep -En -- "$p450_fatal_pattern" "$p450_candidate_log" \
      >&2 || true
    return 1
  fi
  p450_startup_success_count="$(
    /usr/bin/awk '
      index($0, "Startup script returned successfully") { count += 1 }
      END { print count + 0 }
    ' "$p450_candidate_log"
  )"
  if [[ "$p450_startup_success_count" != "1" ]]; then
    echo "expected exactly one successful PX4 startup, observed $p450_startup_success_count" >&2
    return 1
  fi
}

if [[ "${1:-}" == "--test-launch-log" ]]; then
  [[ $# -eq 2 ]] || exit 64
  if p450_launch_log_is_clean "$2"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_plugin_maps_is_exact() {
  local p450_expected_plugin="$1"
  local p450_external_plugin="$2"
  local p450_maps_file="$3"

  if ! p450_expected_plugin="$({
    /usr/bin/realpath -e -- "$p450_expected_plugin"
  } 2>/dev/null)" || [[ ! -f "$p450_expected_plugin" ]]; then
    echo "expected SIM plugin must be a readable file" >&2
    return 66
  fi
  if ! p450_external_plugin="$({
    /usr/bin/realpath -e -- "$p450_external_plugin"
  } 2>/dev/null)" || [[ ! -f "$p450_external_plugin" ]]; then
    echo "external PX4 plugin must be a readable file" >&2
    return 66
  fi
  if [[ ! -f "$p450_maps_file" || ! -r "$p450_maps_file" ]]; then
    echo "process maps must name a readable regular file" >&2
    return 66
  fi
  if [[ "${p450_expected_plugin##*/}" != \
      "${p450_external_plugin##*/}" ]]; then
    echo "SIM and external plugin basenames must match" >&2
    return 64
  fi

  /usr/bin/env -i \
    PATH=/usr/bin:/bin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - \
      "$p450_expected_plugin" "$p450_maps_file" <<'PY'
import os
import re
import sys
from pathlib import Path

expected = Path(sys.argv[1]).resolve(strict=True)
maps_path = Path(sys.argv[2])
mapped = set()

def decode_proc_path(value):
    return re.sub(
        r"\\([0-7]{3})",
        lambda match: chr(int(match.group(1), 8)),
        value,
    )

with maps_path.open("r", encoding="utf-8", errors="surrogateescape") as stream:
    for line in stream:
        fields = line.rstrip("\n").split(None, 5)
        if len(fields) != 6:
            continue
        pathname = decode_proc_path(fields[5])
        if os.path.basename(pathname) == expected.name:
            mapped.add(Path(pathname).resolve(strict=False))

if mapped != {expected}:
    print(
        "expected exactly %s, observed %s" %
        (expected, ", ".join(str(path) for path in sorted(mapped))),
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
}

if [[ "${1:-}" == "--test-plugin-maps" ]]; then
  [[ $# -eq 4 ]] || exit 64
  if p450_plugin_maps_is_exact "$2" "$3" "$4"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_tf_authorities_are_exact() {
  local p450_authorities_file="$1"
  if [[ ! -f "$p450_authorities_file" || ! -r "$p450_authorities_file" ]]; then
    echo "TF authorities must name a readable regular file" >&2
    return 66
  fi

  /usr/bin/env -i \
    PATH=/usr/bin:/bin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$p450_authorities_file" <<'PY'
import json
import sys
from pathlib import Path

expected = {
    "uav1/base_link": {"/uav_control_main_1"},
    "uav1/camera_link": {"/uav_control_main_1"},
    "uav1/camera_depth_frame": {"/uav1/p450_tf_camera_depth"},
    "uav1/camera_ired1_frame": {"/uav1/p450_tf_camera_ired1"},
    "uav1/camera_ired2_frame": {"/uav1/p450_tf_camera_ired2"},
    "uav1/camera_imu_link": {"/uav1/p450_tf_camera_imu"},
    "uav1/d435i_link": {"/uav1/p450_tf_camera_d435i"},
    "uav1/camera_color_optical_frame": {
        "/uav1/p450_tf_camera_color_optical"},
    "uav1/camera_depth_optical_frame": {"/uav1/p450_tf_depth_optical"},
}
forbidden = {"map_base_link", "base_link", "uav1/base_link_gt"}

try:
    raw = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or not raw:
        raise ValueError("top level must be a non-empty object")
    authorities = {}
    for child, callers in raw.items():
        if (not isinstance(child, str) or not child or
                not isinstance(callers, list) or not callers or
                any(not isinstance(caller, str) or not caller
                    for caller in callers) or
                len(callers) != len(set(callers))):
            raise ValueError("invalid authority entry for %r" % child)
        authorities[child] = set(callers)
except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
    print("invalid TF authority capture: %s" % error, file=sys.stderr)
    raise SystemExit(65)

errors = []
for child, callers in sorted(authorities.items()):
    print("%s <- %s" % (child, ",".join(sorted(callers))))
    if len(callers) > 1:
        errors.append("duplicate TF child %s authorities=%s" %
                      (child, sorted(callers)))
for child, wanted in expected.items():
    observed = authorities.get(child, set())
    if observed != wanted:
        errors.append("%s authorities=%s expected=%s" %
                      (child, sorted(observed), sorted(wanted)))
for child in sorted(forbidden & set(authorities)):
    errors.append("forbidden TF child %s authorities=%s" %
                  (child, sorted(authorities[child])))
if errors:
    print("; ".join(errors), file=sys.stderr)
    raise SystemExit(1)
PY
}

if [[ "${1:-}" == "--test-tf-authorities" ]]; then
  [[ $# -eq 2 ]] || exit 64
  if p450_tf_authorities_are_exact "$2"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_validate_render_capability() {
  local p450_display="$1"
  local p450_xauthority="$2"
  local p450_socket_root="$3"
  local p450_display_number

  p450_validated_display=""
  p450_validated_xauthority=""
  if [[ ! "$p450_display" =~ ^:[0-9]+([.][0-9]+)?$ ]]; then
    echo "render DISPLAY must name a local X display like :0 or :0.0" >&2
    return 64
  fi
  if ! p450_validated_xauthority="$({
    /usr/bin/realpath -e -- "$p450_xauthority"
  } 2>/dev/null)" || [[ ! -f "$p450_validated_xauthority" || \
      ! -r "$p450_validated_xauthority" ]]; then
    echo "render XAUTHORITY must name a readable regular file" >&2
    return 66
  fi
  if ! p450_socket_root="$({
    /usr/bin/realpath -e -- "$p450_socket_root"
  } 2>/dev/null)" || [[ ! -d "$p450_socket_root" ]]; then
    echo "render X socket root must name an existing directory" >&2
    return 66
  fi

  p450_display_number="${p450_display#:}"
  p450_display_number="${p450_display_number%%.*}"
  if [[ ! -S "$p450_socket_root/X$p450_display_number" ]]; then
    echo "local X display socket is unavailable for $p450_display" >&2
    return 69
  fi
  p450_validated_display="$p450_display"
}

if [[ "${1:-}" == "--test-render-capability" ]]; then
  [[ $# -eq 4 ]] || exit 64
  if p450_validate_render_capability "$2" "$3" "$4"; then
    printf 'DISPLAY=%s\nXAUTHORITY=%s\n' \
      "$p450_validated_display" "$p450_validated_xauthority"
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_write_result_manifest() {
  [[ $# -eq 9 ]] || return 64
  /usr/bin/env -i \
    PATH=/usr/bin:/bin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$@" <<'PY'
import json
import os
import sys
from pathlib import Path


def parse_status(raw, *, optional=False):
    if optional and raw == "":
        return None
    try:
        value = int(raw)
    except ValueError as error:
        raise ValueError("status must be an integer") from error
    if not 0 <= value <= 255 or str(value) != raw:
        raise ValueError("status must be canonical and in [0, 255]")
    return value


result_path = Path(sys.argv[1])
verdict = sys.argv[2]
script_status = parse_status(sys.argv[3])
launcher_status = parse_status(sys.argv[4], optional=True)
shutdown_escalated_raw = sys.argv[5]
run_id = sys.argv[6]
px4_workdir = sys.argv[7]
gps_plugin = Path(sys.argv[8]).resolve(strict=True)
groundtruth_plugin = Path(sys.argv[9]).resolve(strict=True)

if verdict not in {"PASS", "FAIL"}:
    raise ValueError("verdict must be PASS or FAIL")
if (verdict == "PASS") != (script_status == 0):
    raise ValueError("verdict and script status disagree")
if shutdown_escalated_raw not in {"0", "1"}:
    raise ValueError("shutdown_escalated must be 0 or 1")
if not run_id or not px4_workdir:
    raise ValueError("run identity fields must be non-empty")
if not gps_plugin.is_file() or not groundtruth_plugin.is_file():
    raise ValueError("plugin paths must name regular files")
if result_path.exists() or not result_path.parent.is_dir():
    raise ValueError("result path must be new and have an existing parent")

payload = {
    "schema_version": 1,
    "verdict": verdict,
    "script_status": script_status,
    "launcher_status": launcher_status,
    "shutdown_escalated": shutdown_escalated_raw == "1",
    "run_id": run_id,
    "px4_workdir": px4_workdir,
    "plugins": {
        "gps": str(gps_plugin),
        "groundtruth": str(groundtruth_plugin),
    },
}
temporary_path = result_path.with_name(".%s.%d.tmp" %
                                       (result_path.name, os.getpid()))
try:
    with temporary_path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, result_path)
except BaseException:
    try:
        temporary_path.unlink()
    except FileNotFoundError:
        pass
    raise
PY
}

if [[ "${1:-}" == "--test-result-manifest" ]]; then
  [[ $# -eq 10 ]] || exit 64
  if p450_write_result_manifest \
      "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
p450_script_dir="${p450_script_path%/*}"
p450_repo_root="${p450_script_dir%/*}"
p450_environment_wrapper="$p450_repo_root/scripts/with_p450_env.bash"

if [[ -n "${P450_GAZEBO_DISPLAY:-}" || \
      -n "${P450_GAZEBO_XAUTHORITY:-}" ]]; then
  p450_render_display="${P450_GAZEBO_DISPLAY:-}"
  p450_render_xauthority="${P450_GAZEBO_XAUTHORITY:-}"
else
  p450_render_display="${DISPLAY:-}"
  p450_render_xauthority="${XAUTHORITY:-}"
fi
if ! p450_validate_render_capability \
    "$p450_render_display" "$p450_render_xauthority" /tmp/.X11-unix; then
  p450_die "D435 rendering requires a usable local X display capability"
fi
p450_render_display="$p450_validated_display"
p450_render_xauthority="$p450_validated_xauthority"

p450_ros_port="${P450_SMOKE_ROS_PORT:-11361}"
p450_gazebo_port="${P450_SMOKE_GAZEBO_PORT:-11362}"
p450_startup_seconds="${P450_SMOKE_STARTUP_SECONDS:-150}"
p450_topic_seconds="${P450_SMOKE_TOPIC_SECONDS:-20}"
p450_observation_seconds="${P450_SMOKE_OBSERVATION_SECONDS:-10}"
p450_total_seconds="${P450_SMOKE_TOTAL_SECONDS:-300}"

p450_validate_integer() {
  local p450_name="$1"
  local p450_value="$2"
  local p450_minimum="$3"
  local p450_maximum="$4"
  if [[ ! "$p450_value" =~ ^[0-9]+$ ]] || \
     (( p450_value < p450_minimum || p450_value > p450_maximum )); then
    p450_die "$p450_name must be an integer in [$p450_minimum, $p450_maximum]"
  fi
}

p450_validate_integer P450_SMOKE_ROS_PORT "$p450_ros_port" 1024 65535
p450_validate_integer P450_SMOKE_GAZEBO_PORT "$p450_gazebo_port" 1024 65535
p450_validate_integer P450_SMOKE_STARTUP_SECONDS "$p450_startup_seconds" 30 900
p450_validate_integer P450_SMOKE_TOPIC_SECONDS "$p450_topic_seconds" 5 120
p450_validate_integer P450_SMOKE_OBSERVATION_SECONDS "$p450_observation_seconds" 5 120
p450_validate_integer P450_SMOKE_TOTAL_SECONDS "$p450_total_seconds" 60 1200

if (( p450_total_seconds < p450_startup_seconds + p450_observation_seconds + 20 )); then
  p450_die "P450_SMOKE_TOTAL_SECONDS is too short for startup and observation"
fi
if [[ "$p450_ros_port" == "$p450_gazebo_port" ]]; then
  p450_die "ROS and Gazebo master ports must differ"
fi

if [[ "${P450_SMOKE_INNER:-0}" != "1" ]]; then
  [[ -x "$p450_environment_wrapper" ]] || \
    p450_die "runtime environment wrapper is missing or not executable"
  export P450_GAZEBO_DISPLAY="$p450_render_display"
  export P450_GAZEBO_XAUTHORITY="$p450_render_xauthority"
  exec "$p450_environment_wrapper" /usr/bin/env \
    P450_SMOKE_INNER=1 \
    P450_SMOKE_ROS_PORT="$p450_ros_port" \
    P450_SMOKE_GAZEBO_PORT="$p450_gazebo_port" \
    P450_SMOKE_STARTUP_SECONDS="$p450_startup_seconds" \
    P450_SMOKE_TOPIC_SECONDS="$p450_topic_seconds" \
    P450_SMOKE_OBSERVATION_SECONDS="$p450_observation_seconds" \
    P450_SMOKE_TOTAL_SECONDS="$p450_total_seconds" \
    ROS_MASTER_URI="http://127.0.0.1:$p450_ros_port" \
    GAZEBO_MASTER_URI="http://127.0.0.1:$p450_gazebo_port" \
    "$p450_script_path"
fi

[[ "${ROS_MASTER_URI:-}" == "http://127.0.0.1:$p450_ros_port" ]] || \
  p450_die "inner ROS_MASTER_URI is not the dedicated loopback endpoint"
[[ "${GAZEBO_MASTER_URI:-}" == "http://127.0.0.1:$p450_gazebo_port" ]] || \
  p450_die "inner GAZEBO_MASTER_URI is not the dedicated loopback endpoint"
[[ "${ROS_HOME:-}" == "$p450_repo_root/logs/ros" ]] || \
  p450_die "inner runtime was not created by with_p450_env.bash"
[[ -n "${P450_PX4_ROOT:-}" ]] || p450_die "P450_PX4_ROOT is unavailable"

declare -A p450_seen_ports=()
for p450_port in \
  "$p450_ros_port" "$p450_gazebo_port" 4560 14540 14560 14580; do
  if [[ -n "${p450_seen_ports[$p450_port]:-}" ]]; then
    p450_die "runtime port $p450_port is assigned more than once"
  fi
  p450_seen_ports[$p450_port]=1
done

p450_check_port_free() {
  local p450_port="$1"
  local p450_protocol
  local p450_socket_state
  for p450_protocol in t u; do
    if ! p450_socket_state="$(
      /usr/bin/ss -H -"${p450_protocol}"an "sport = :$p450_port" 2>&1
    )"; then
      p450_die "cannot inspect ${p450_protocol} port $p450_port: $p450_socket_state"
    fi
    if [[ -n "$p450_socket_state" ]]; then
      p450_die "${p450_protocol} port $p450_port is already in use: $p450_socket_state"
    fi
  done
}

for p450_port in \
  "$p450_ros_port" "$p450_gazebo_port" 4560 14540 14560 14580; do
  p450_check_port_free "$p450_port"
done

p450_run_id="$(/usr/bin/date -u +%Y%m%dT%H%M%SZ)-$$"
p450_log_dir="$p450_repo_root/logs/p450_standalone/$p450_run_id"
/usr/bin/mkdir -p -- "$p450_log_dir"
p450_launch_log="$p450_log_dir/roslaunch.log"
p450_px4_workdir="sitl_smoke_$p450_run_id"
p450_px4_work_path="$ROS_HOME/$p450_px4_workdir"
[[ ! -e "$p450_px4_work_path" ]] || \
  p450_die "run-local PX4 workdir already exists: $p450_px4_work_path"

p450_launch_pid=""
p450_launch_pgid=""
p450_roslaunch_pid=""
p450_gzserver_seen=0
p450_px4_seen=0
p450_group_is_alive() {
  [[ "$p450_launch_pgid" =~ ^[1-9][0-9]*$ ]] || return 1
  /usr/bin/ps -eo pgid=,stat= | /usr/bin/awk \
    -v pgid="$p450_launch_pgid" '
      $1 == pgid && $2 !~ /^Z/ { found = 1 }
      END { exit(found ? 0 : 1) }
    '
}

p450_wait_for_group_exit() {
  local p450_attempt
  local p450_attempts="$1"
  [[ "$p450_attempts" =~ ^[1-9][0-9]*$ ]] || return 64
  for p450_attempt in $(/usr/bin/seq 1 "$p450_attempts"); do
    if ! p450_group_is_alive; then
      return 0
    fi
    /usr/bin/sleep 0.5
  done
  return 1
}

p450_pid_is_alive() {
  local p450_pid="$1"
  local p450_state
  [[ "$p450_pid" =~ ^[1-9][0-9]*$ ]] || return 1
  if ! p450_state="$(/usr/bin/ps -o stat= -p "$p450_pid" 2>/dev/null)"; then
    return 1
  fi
  [[ -n "$p450_state" && "$p450_state" != Z* ]]
}

p450_wait_for_pid_exit() {
  local p450_pid="$1"
  local p450_attempt
  local p450_attempts="${2:-60}"
  for p450_attempt in $(/usr/bin/seq 1 "$p450_attempts"); do
    if ! p450_pid_is_alive "$p450_pid"; then
      return 0
    fi
    /usr/bin/sleep 0.5
  done
  return 1
}

p450_find_roslaunch_pid_in_group() {
  local p450_candidate_pid
  local p450_candidate_pgid
  local p450_candidate_state
  local p450_candidate_executable
  local p450_python_executable
  local -a p450_candidate_argv=()
  local -a p450_candidates=()

  [[ "$p450_launch_pgid" =~ ^[1-9][0-9]*$ ]] || return 1
  p450_python_executable="$(/usr/bin/realpath -e -- /usr/bin/python3)" || \
    return 1
  while read -r p450_candidate_pid p450_candidate_pgid \
      p450_candidate_state; do
    [[ "$p450_candidate_pid" =~ ^[1-9][0-9]*$ ]] || continue
    [[ "$p450_candidate_pgid" == "$p450_launch_pgid" ]] || continue
    [[ "$p450_candidate_state" != Z* ]] || continue
    if ! p450_candidate_executable="$({
      /usr/bin/realpath -e -- "/proc/$p450_candidate_pid/exe"
    } 2>/dev/null)" || \
        [[ "$p450_candidate_executable" != "$p450_python_executable" ]]; then
      continue
    fi
    p450_candidate_argv=()
    mapfile -d '' -t p450_candidate_argv \
      <"/proc/$p450_candidate_pid/cmdline" 2>/dev/null || true
    if (( ${#p450_candidate_argv[@]} >= 2 )) && \
        [[ "${p450_candidate_argv[1]}" == \
          "/opt/ros/noetic/bin/roslaunch" ]]; then
      p450_candidates+=("$p450_candidate_pid")
    fi
  done < <(/usr/bin/ps -ww -eo pid=,pgid=,stat=)
  if (( ${#p450_candidates[@]} != 1 )); then
    return 1
  fi
  printf '%s\n' "${p450_candidates[0]}"
}

p450_cleanup() {
  local p450_status="$?"
  local p450_launch_status=""
  local p450_shutdown_escalated=0
  local p450_verdict
  trap - EXIT INT TERM
  if p450_group_is_alive; then
    if [[ -z "$p450_roslaunch_pid" ]]; then
      p450_roslaunch_pid="$(p450_find_roslaunch_pid_in_group)" || true
    fi
    if p450_pid_is_alive "$p450_roslaunch_pid"; then
      if ! kill -INT "$p450_roslaunch_pid" 2>/dev/null; then
        echo "P450 standalone smoke: could not signal roslaunch" >&2
        p450_status=1
      elif ! p450_wait_for_pid_exit "$p450_roslaunch_pid" 60; then
        p450_shutdown_escalated=1
      fi
    else
      echo "P450 standalone smoke: roslaunch disappeared before shutdown" >&2
      p450_status=1
    fi
  fi
  if p450_group_is_alive; then
    p450_shutdown_escalated=1
    kill -TERM -- -"$p450_launch_pgid" 2>/dev/null || true
    if ! p450_wait_for_group_exit 40; then
      kill -KILL -- -"$p450_launch_pgid" 2>/dev/null || true
      if ! p450_wait_for_group_exit 40; then
        echo "P450 standalone smoke: owned process group ignored SIGKILL" >&2
        p450_status=1
      fi
    fi
  fi
  if [[ -n "$p450_launch_pid" ]]; then
    if p450_pid_is_alive "$p450_launch_pid"; then
      echo "P450 standalone smoke: owned launcher is still running; refusing to wait" >&2
      p450_status=1
    else
      if wait "$p450_launch_pid" 2>/dev/null; then
        p450_launch_status=0
      else
        p450_launch_status="$?"
      fi
      if (( p450_launch_status != 0 )); then
        echo "P450 standalone smoke: launcher exited $p450_launch_status" >&2
        p450_status=1
      fi
    fi
  fi
  if (( p450_shutdown_escalated )); then
    echo "P450 standalone smoke: shutdown required signal escalation" >&2
    p450_status=1
  fi
  if ! p450_launch_log_is_clean "$p450_launch_log"; then
    echo "P450 standalone smoke: fatal runtime event in launch log" >&2
    p450_status=1
  fi
  if p450_group_is_alive; then
    echo "P450 standalone smoke: owned process group did not terminate" >&2
    p450_status=1
  fi
  if (( p450_status == 0 )); then
    p450_verdict=PASS
  else
    p450_verdict=FAIL
  fi
  if ! p450_write_result_manifest \
      "$p450_log_dir/result.json" \
      "$p450_verdict" \
      "$p450_status" \
      "$p450_launch_status" \
      "$p450_shutdown_escalated" \
      "$p450_run_id" \
      "$p450_px4_workdir" \
      "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_gps_plugin.so" \
      "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_groundtruth_plugin.so"; then
    echo "P450 standalone smoke: could not write result.json" >&2
    p450_status=1
  fi
  if (( p450_status == 0 )); then
    echo "P450 standalone smoke PASS; logs: $p450_log_dir"
  else
    echo "P450 standalone smoke FAIL ($p450_status); logs: $p450_log_dir" >&2
  fi
  exit "$p450_status"
}

trap p450_cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

/usr/bin/setsid --wait \
  /usr/bin/timeout --signal=INT --kill-after=15s "${p450_total_seconds}s" \
  /opt/ros/noetic/bin/roslaunch \
    --screen --sigint-timeout=10 --sigterm-timeout=5 \
    sim_platform_bringup p450_standalone.launch \
    gui:=false use_sim_time:=true px4_workdir:="$p450_px4_workdir" \
  >"$p450_launch_log" 2>&1 &
p450_launch_pid="$!"

p450_attempt=0
while (( p450_attempt < 5 )); do
  ((p450_attempt += 1))
  if ! kill -0 "$p450_launch_pid" 2>/dev/null; then
    /usr/bin/tail -n 120 "$p450_launch_log" >&2 || true
    p450_die "roslaunch exited before its process group was established"
  fi
  p450_launch_pgid="$(
    /usr/bin/ps -o pgid= -p "$p450_launch_pid" | /usr/bin/tr -d '[:space:]'
  )"
  [[ "$p450_launch_pgid" =~ ^[0-9]+$ ]] && break
  /usr/bin/sleep 0.2
done

p450_script_pgid="$(/usr/bin/ps -o pgid= -p "$$" | /usr/bin/tr -d '[:space:]')"
[[ "$p450_launch_pgid" =~ ^[0-9]+$ ]] || \
  p450_die "could not resolve the owned roslaunch process group"
(( p450_launch_pgid > 1 )) || p450_die "unsafe roslaunch process group"
[[ "$p450_launch_pgid" != "$p450_script_pgid" ]] || \
  p450_die "roslaunch did not enter an isolated process group"

for p450_attempt in $(/usr/bin/seq 1 50); do
  if p450_roslaunch_pid="$(p450_find_roslaunch_pid_in_group)"; then
    break
  fi
  kill -0 "$p450_launch_pid" 2>/dev/null || break
  /usr/bin/sleep 0.1
done
[[ "$p450_roslaunch_pid" =~ ^[1-9][0-9]*$ ]] || \
  p450_die "could not identify the owned roslaunch process"

p450_fail_if_launch_stopped() {
  if ! p450_group_is_alive || ! kill -0 "$p450_launch_pid" 2>/dev/null; then
    /usr/bin/tail -n 160 "$p450_launch_log" >&2 || true
    p450_die "the owned roslaunch process group stopped unexpectedly"
  fi
  if (( p450_gzserver_seen )) && ! p450_launch_has_process gzserver; then
    /usr/bin/tail -n 160 "$p450_launch_log" >&2 || true
    p450_die "gzserver stopped unexpectedly"
  fi
  if (( p450_px4_seen )) && ! p450_launch_has_process \
      "$P450_PX4_ROOT/build/amovlab_sitl_default/bin/px4"; then
    /usr/bin/tail -n 160 "$p450_launch_log" >&2 || true
    p450_die "PX4 /uav1/sitl_1 stopped unexpectedly"
  fi
}

p450_try_master() {
  /usr/bin/timeout --signal=TERM --kill-after=2s 3s \
    /opt/ros/noetic/bin/rosparam get /use_sim_time 2>/dev/null | \
    /usr/bin/grep -qx true
}

p450_try_node() {
  local p450_node="$1"
  /usr/bin/timeout --signal=TERM --kill-after=2s 3s \
    /opt/ros/noetic/bin/rosnode ping -c 1 "$p450_node" >/dev/null 2>&1
}

p450_launch_has_process() {
  local p450_needle="$1"
  kill -0 "$p450_launch_pid" 2>/dev/null || return 1
  /usr/bin/ps -ww -eo pid=,ppid=,pgid=,stat=,args= | \
    p450_snapshot_has_live_descendant "$p450_launch_pid" "$p450_needle"
}

p450_snapshot_live_descendant_pids() {
  local p450_root_pid="$1"
  [[ "$p450_root_pid" =~ ^[1-9][0-9]*$ ]] || return 64
  [[ "$p450_root_pid" != "1" ]] || return 64
  /usr/bin/awk -v root_pid="$p450_root_pid" '
    NF >= 3 && $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/ {
      parent[$1] = $2
      state[$1] = $3
    }
    END {
      for (pid in parent) {
        if (pid == root_pid || state[pid] ~ /^Z/)
          continue
        generation += 1
        cursor = pid
        while ((cursor in parent) && seen[cursor] != generation) {
          seen[cursor] = generation
          cursor = parent[cursor]
          if (cursor == root_pid) {
            print pid
            break
          }
        }
      }
    }
  '
}

p450_find_unique_gzserver_pid() {
  local p450_candidate_pid
  local p450_candidate_exe
  local p450_gzserver_executable
  local -a p450_gzserver_pids=()
  p450_gzserver_executable="$(/usr/bin/realpath -e -- /usr/bin/gzserver)" || \
    return 1
  while IFS= read -r p450_candidate_pid; do
    [[ "$p450_candidate_pid" =~ ^[1-9][0-9]*$ ]] || continue
    if p450_candidate_exe="$({
      /usr/bin/realpath -e -- "/proc/$p450_candidate_pid/exe"
    } 2>/dev/null)" && \
        [[ "$p450_candidate_exe" == "$p450_gzserver_executable" ]]; then
      p450_gzserver_pids+=("$p450_candidate_pid")
    fi
  done < <(
    /usr/bin/ps -ww -eo pid=,ppid=,stat= | \
      p450_snapshot_live_descendant_pids "$p450_launch_pid"
  )
  if (( ${#p450_gzserver_pids[@]} != 1 )); then
    echo "expected one /usr/bin/gzserver descendant, found ${#p450_gzserver_pids[@]}" >&2
    return 1
  fi
  printf '%s\n' "${p450_gzserver_pids[0]}"
}

p450_try_exact_model() {
  local p450_response
  if ! p450_response="$(
    /usr/bin/timeout --signal=TERM --kill-after=2s 5s \
      /opt/ros/noetic/bin/rosservice call \
      /gazebo/get_world_properties "{}" 2>/dev/null
  )"; then
    return 1
  fi
  /usr/bin/python3 -c '
import sys
import yaml
response = yaml.safe_load(sys.stdin.read())
models = response.get("model_names", []) if isinstance(response, dict) else []
if response.get("success") is not True or models.count("p450_D435i_0") != 1:
    raise SystemExit(1)
' <<<"$p450_response"
}

p450_wait_until() {
  local p450_label="$1"
  local p450_attempt_log="$2"
  shift 2
  local p450_deadline=$(( $(/usr/bin/date +%s) + p450_startup_seconds ))
  while (( $(/usr/bin/date +%s) < p450_deadline )); do
    p450_fail_if_launch_stopped
    if "$@" >"$p450_attempt_log" 2>&1; then
      return 0
    fi
    /usr/bin/sleep 1
  done
  [[ ! -s "$p450_attempt_log" ]] || /usr/bin/tail -n 80 "$p450_attempt_log" >&2
  /usr/bin/tail -n 160 "$p450_launch_log" >&2 || true
  p450_die "timed out waiting for $p450_label"
}

p450_wait_until "dedicated ROS master" "$p450_log_dir/master.wait.log" \
  p450_try_master
p450_wait_until "Gazebo node" "$p450_log_dir/gazebo.wait.log" \
  p450_try_node /gazebo
p450_wait_until "MAVROS node /uav1/mavros" "$p450_log_dir/mavros-node.wait.log" \
  p450_try_node /uav1/mavros
p450_wait_until "controller node /uav_control_main_1" \
  "$p450_log_dir/controller-node.wait.log" \
  p450_try_node /uav_control_main_1
p450_wait_until "gzserver process" "$p450_log_dir/gzserver.wait.log" \
  p450_launch_has_process gzserver
p450_gzserver_seen=1
p450_wait_until "PX4 process /uav1/sitl_1" "$p450_log_dir/px4.wait.log" \
  p450_launch_has_process "$P450_PX4_ROOT/build/amovlab_sitl_default/bin/px4"
p450_px4_seen=1
p450_wait_until "exactly one p450_D435i_0 model" \
  "$p450_log_dir/model.wait.log" p450_try_exact_model

if ! p450_gzserver_pid="$(p450_find_unique_gzserver_pid)"; then
  p450_die "could not identify the exact gzserver process"
fi
if ! /usr/bin/cp -- "/proc/$p450_gzserver_pid/maps" \
    "$p450_log_dir/gzserver.maps"; then
  p450_die "could not capture gzserver plugin mappings"
fi
for p450_overlay_plugin in \
    libgazebo_gps_plugin.so libgazebo_groundtruth_plugin.so; do
  if ! p450_plugin_maps_is_exact \
      "$p450_repo_root/install/p450-runtime-overlays/lib/$p450_overlay_plugin" \
      "$P450_PX4_ROOT/build/amovlab_sitl_default/build_gazebo/$p450_overlay_plugin" \
      "$p450_log_dir/gzserver.maps"; then
    p450_die "gzserver did not load exactly the SIM $p450_overlay_plugin overlay"
  fi
done

p450_check_state() {
  local p450_topic="$1"
  local p450_filter="$2"
  local p450_output="$3"
  if ! /usr/bin/timeout --signal=TERM --kill-after=2s \
      "${p450_topic_seconds}s" \
      /opt/ros/noetic/bin/rostopic echo --noarr -n 1 \
      --filter="$p450_filter" "$p450_topic" >"$p450_output" 2>&1; then
    /usr/bin/tail -n 80 "$p450_output" >&2 || true
    p450_die "invalid or missing state on $p450_topic"
  fi
}

p450_check_state /uav1/mavros/state \
  'm.connected and not m.armed' "$p450_log_dir/mavros-state.log"
p450_check_state /uav1/prometheus/state \
  'm.connected and not m.armed and m.odom_valid' \
  "$p450_log_dir/prometheus-state.log"

p450_check_topic_samples() {
  local p450_topic="$1"
  local p450_expected_frame="$2"
  local p450_output="$3"
  local p450_filter
  p450_filter="m.header.stamp.to_nsec() > 0 and bool(m.header.frame_id) and m.header.frame_id.lstrip('/') == '$p450_expected_frame'"
  if ! /usr/bin/timeout --signal=TERM --kill-after=2s \
      "${p450_topic_seconds}s" \
      /opt/ros/noetic/bin/rostopic echo --noarr -n 2 \
      --filter="$p450_filter" "$p450_topic" >"$p450_output" 2>&1; then
    /usr/bin/tail -n 80 "$p450_output" >&2 || true
    p450_die "missing timestamped samples on $p450_topic"
  fi
}

p450_check_topic_samples /uav1/camera/color/image_raw uav1/camera_link \
  "$p450_log_dir/color-image.log"
p450_check_topic_samples /uav1/camera/color/camera_info uav1/camera_link \
  "$p450_log_dir/color-info.log"
p450_check_topic_samples /uav1/camera/depth/image_raw uav1/camera_depth_frame \
  "$p450_log_dir/depth-image.log"
p450_check_topic_samples /uav1/camera/depth/camera_info uav1/camera_depth_frame \
  "$p450_log_dir/depth-info.log"
p450_check_topic_samples /uav1/camera/imu uav1/camera_imu_link \
  "$p450_log_dir/imu.log"

if ! /usr/bin/timeout --signal=TERM --kill-after=2s 25s \
    /usr/bin/python3 - >"$p450_log_dir/tf-current.log" 2>&1 <<'PY'
import time

import rospy
import tf2_ros
from rosgraph_msgs.msg import Clock

rospy.init_node("p450_smoke_tf_current", anonymous=True, disable_signals=True)
rospy.wait_for_message("/clock", Clock, timeout=10.0)
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
listener = tf2_ros.TransformListener(buffer)
frames = (
    "uav1/base_link",
    "uav1/camera_link",
    "uav1/camera_depth_frame",
    "uav1/camera_ired1_frame",
    "uav1/camera_ired2_frame",
    "uav1/camera_imu_link",
    "uav1/d435i_link",
    "uav1/camera_color_optical_frame",
    "uav1/camera_depth_optical_frame",
)
for frame in frames:
    deadline = time.monotonic() + 8.0
    last_error = None
    while time.monotonic() < deadline:
        try:
            transform = buffer.lookup_transform(
                "world", frame, rospy.Time(0), rospy.Duration(0.5))
            now = rospy.Time.now()
            stamp = transform.header.stamp
            age = (now - stamp).to_sec()
            if now.to_sec() <= 0.0 or stamp.to_sec() <= 0.0:
                raise RuntimeError("simulation or transform time is zero")
            if age < -0.2 or age > 2.0:
                raise RuntimeError("transform age %.6f is not current" % age)
            print("world -> %s stamp=%.9f age=%.6f" %
                  (frame, stamp.to_sec(), age))
            break
        except Exception as error:  # tf2 exceptions share this retry path.
            last_error = error
            time.sleep(0.1)
    else:
        raise SystemExit("cannot resolve current world -> %s: %s" %
                         (frame, last_error))
del listener
PY
then
  /usr/bin/tail -n 100 "$p450_log_dir/tf-current.log" >&2 || true
  p450_die "required current-time TF chain did not resolve"
fi

if ! /usr/bin/timeout --signal=TERM --kill-after=2s 15s \
    /usr/bin/python3 - "$p450_log_dir/tf-authorities.json" \
    >"$p450_log_dir/tf-authority-collect.log" 2>&1 <<'PY'
import collections
import json
import sys
import time

import rospy
from tf2_msgs.msg import TFMessage

authorities = collections.defaultdict(set)

def collect(message):
    caller = getattr(message, "_connection_header", {}).get(
        "callerid", "<unknown>")
    for transform in message.transforms:
        child = transform.child_frame_id.lstrip("/")
        authorities[child].add(caller)

rospy.init_node("p450_smoke_tf_authority", anonymous=True, disable_signals=True)
subscribers = (
    rospy.Subscriber("/tf", TFMessage, collect, queue_size=200),
    rospy.Subscriber("/tf_static", TFMessage, collect, queue_size=200),
)
deadline = time.monotonic() + 5.0
while time.monotonic() < deadline and not rospy.is_shutdown():
    time.sleep(0.05)

del subscribers
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump(
        {child: sorted(callers) for child, callers in authorities.items()},
        stream,
        sort_keys=True,
    )
PY
then
  /usr/bin/tail -n 120 "$p450_log_dir/tf-authority-collect.log" >&2 || true
  p450_die "TF child authority collection failed"
fi
if ! p450_tf_authorities_are_exact \
    "$p450_log_dir/tf-authorities.json" \
    >"$p450_log_dir/tf-authority.log" 2>&1; then
  /usr/bin/tail -n 120 "$p450_log_dir/tf-authority.log" >&2 || true
  p450_die "TF child authority contract failed"
fi

p450_observation_deadline=$((
  $(/usr/bin/date +%s) + p450_observation_seconds
))
while (( $(/usr/bin/date +%s) < p450_observation_deadline )); do
  p450_fail_if_launch_stopped
  p450_try_node /gazebo || p450_die "Gazebo node stopped during observation"
  p450_try_node /uav1/mavros || p450_die "MAVROS stopped during observation"
  p450_try_node /uav_control_main_1 || \
    p450_die "controller stopped during observation"
  p450_launch_has_process gzserver || \
    p450_die "gzserver stopped during observation"
  p450_launch_has_process \
    "$P450_PX4_ROOT/build/amovlab_sitl_default/bin/px4" || \
    p450_die "PX4 /uav1/sitl_1 stopped during observation"
  p450_try_exact_model || \
    p450_die "P450 model multiplicity changed during observation"
  /usr/bin/sleep 1
done
