#!/bin/bash

set -euo pipefail
umask 077

p450_die() {
  echo "P450 standalone smoke: $*" >&2
  exit 1
}

if [[ "${P450_SENSOR_PROFILE+x}" == "x" ]]; then
  p450_sensor_profile="$P450_SENSOR_PROFILE"
else
  p450_sensor_profile="d435"
fi
case "$p450_sensor_profile" in
  d435)
    p450_profile_log_root="logs/p450_standalone"
    p450_profile_launch_args=()
    ;;
  mid360)
    p450_profile_log_root="logs/p450_mid360"
    p450_profile_launch_args=("enable_mid360:=true")
    ;;
  *)
    echo "P450_SENSOR_PROFILE must be exactly d435 or mid360" >&2
    exit 64
    ;;
esac
P450_SENSOR_PROFILE="$p450_sensor_profile"
export P450_SENSOR_PROFILE

if [[ "${1:-}" == "--test-sensor-profile" ]]; then
  [[ $# -eq 1 ]] || exit 64
  if [[ "$p450_sensor_profile" == "mid360" ]]; then
    printf '%s\n' '{"sensor_profile":"mid360","log_root":"logs/p450_mid360","launch_extra_args":["enable_mid360:=true"]}'
  else
    printf '%s\n' '{"sensor_profile":"d435","log_root":"logs/p450_standalone","launch_extra_args":[]}'
  fi
  exit 0
fi

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
  if [[ ! "$p450_value" =~ ^(0|[1-9][0-9]*)$ ]] || \
     (( p450_value < p450_minimum || p450_value > p450_maximum )); then
    echo "$p450_name must be a canonical decimal integer in [$p450_minimum, $p450_maximum]" >&2
    return 64
  fi
}

p450_validate_smoke_integers() {
  p450_validate_integer P450_SMOKE_ROS_PORT "$p450_ros_port" 1024 65535 || return
  p450_validate_integer P450_SMOKE_GAZEBO_PORT "$p450_gazebo_port" 1024 65535 || return
  p450_validate_integer P450_SMOKE_STARTUP_SECONDS "$p450_startup_seconds" 30 900 || return
  p450_validate_integer P450_SMOKE_TOPIC_SECONDS "$p450_topic_seconds" 5 120 || return
  p450_validate_integer P450_SMOKE_OBSERVATION_SECONDS "$p450_observation_seconds" 5 120 || return
  p450_validate_integer P450_SMOKE_TOTAL_SECONDS "$p450_total_seconds" 60 1200 || return
  if (( p450_total_seconds < p450_startup_seconds + p450_observation_seconds + 20 )); then
    echo "P450_SMOKE_TOTAL_SECONDS is too short for startup and observation" >&2
    return 64
  fi
  if [[ "$p450_ros_port" == "$p450_gazebo_port" ]]; then
    echo "ROS and Gazebo master ports must differ" >&2
    return 64
  fi
}

if [[ "${1:-}" == "--test-smoke-integers" ]]; then
  [[ $# -eq 7 ]] || exit 64
  p450_ros_port="$2"
  p450_gazebo_port="$3"
  p450_startup_seconds="$4"
  p450_topic_seconds="$5"
  p450_observation_seconds="$6"
  p450_total_seconds="$7"
  p450_validate_smoke_integers
  exit $?
fi

if p450_validate_smoke_integers; then
  :
else
  p450_integer_status="$?"
  exit "$p450_integer_status"
fi

p450_create_private_run_directory() {
  local p450_run_parent="$1"
  local p450_candidate_run_id="$2"
  local p450_run_parent_canonical=""
  local p450_run_leaf=""
  local p450_run_mode=""

  if [[ "$p450_run_parent" != /* || -L "$p450_run_parent" ]] || \
      ! p450_run_parent_canonical="$({
        /usr/bin/realpath -e -- "$p450_run_parent"
      } 2>/dev/null)" || \
      [[ "$p450_run_parent_canonical" != "$p450_run_parent" || \
         ! -d "$p450_run_parent" || ! -w "$p450_run_parent" ]]; then
    echo "run parent must be an absolute canonical writable directory" >&2
    return 66
  fi
  if [[ ! "$p450_candidate_run_id" =~ \
      ^[0-9]{8}T[0-9]{6}Z-[1-9][0-9]*$ ]]; then
    echo "run id must be a canonical UTC timestamp and PID" >&2
    return 64
  fi
  p450_run_leaf="$p450_run_parent/$p450_candidate_run_id"
  if [[ -e "$p450_run_leaf" || -L "$p450_run_leaf" ]]; then
    echo "run directory already exists" >&2
    return 73
  fi
  if ! /usr/bin/mkdir --mode=700 -- "$p450_run_leaf"; then
    echo "could not create the private run directory" >&2
    return 73
  fi
  if [[ ! -d "$p450_run_leaf" || -L "$p450_run_leaf" ]] || \
      ! p450_run_mode="$({
        /usr/bin/stat -c %a -- "$p450_run_leaf"
      } 2>/dev/null)" || [[ "$p450_run_mode" != "700" ]]; then
    echo "run directory is not a private canonical directory" >&2
    return 73
  fi
  printf '%s\n' "$p450_run_leaf"
}

if [[ "${1:-}" == "--test-private-run-directory" ]]; then
  [[ $# -eq 3 ]] || exit 64
  if p450_create_private_run_directory "$2" "$3"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

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
  p450_fatal_pattern='Segmentation fault|Assertion .* failed|Aborted( \(core dumped\))?|Reboot PX4!|process has died|Startup script returned with return value:|cannot get csv file!|cannot read csv file!|Livox frameName must not be empty|Failed to load plugin.*liblivox_laser_gazebo_plugins\.so'

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
        deleted_suffix = " (deleted)"
        is_deleted = pathname.endswith(deleted_suffix)
        candidate = pathname[:-len(deleted_suffix)] if is_deleted else pathname
        if os.path.basename(candidate) == expected.name:
            if is_deleted:
                print("related plugin mapping is deleted: %s" % pathname,
                      file=sys.stderr)
                raise SystemExit(1)
            mapped.add(Path(candidate).resolve(strict=False))

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

p450_mid360_maps_evidence() {
  local p450_expected_plugin="$1"
  local p450_maps_file="$2"
  /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$p450_expected_plugin" "$p450_maps_file" <<'PY'
import json
import os
import re
import sys
from pathlib import Path

try:
    expected = Path(sys.argv[1]).resolve(strict=True)
    maps_file = Path(sys.argv[2])
    if (not expected.is_file() or expected.name !=
            "liblivox_laser_gazebo_plugins.so"):
        raise ValueError("expected plugin is not the installed Livox plugin")
    if not maps_file.is_file():
        raise ValueError("maps capture must be a readable regular file")
    plugins = set()
    protobuf = set()
    with maps_file.open("r", encoding="utf-8", errors="surrogateescape") as stream:
        for line in stream:
            fields = line.rstrip("\n").split(None, 5)
            if len(fields) != 6 or not fields[5].startswith("/"):
                continue
            raw_path = re.sub(
                r"\\([0-7]{3})",
                lambda match: chr(int(match.group(1), 8)), fields[5])
            deleted_suffix = " (deleted)"
            is_deleted = raw_path.endswith(deleted_suffix)
            candidate = (raw_path[:-len(deleted_suffix)]
                         if is_deleted else raw_path)
            candidate_name = os.path.basename(candidate)
            if candidate_name == expected.name:
                if is_deleted:
                    raise ValueError("Livox plugin mapping is deleted")
                path = Path(candidate).resolve(strict=True)
                plugins.add(path)
            if re.match(r"^libprotobuf\.so(?:\.|$)", candidate_name):
                if is_deleted:
                    raise ValueError("protobuf mapping is deleted")
                path = Path(candidate).resolve(strict=True)
                protobuf.add(path)
    if plugins != {expected}:
        raise ValueError("Livox plugin maps must contain only %s, got %s" %
                         (expected, sorted(str(path) for path in plugins)))
    if len(protobuf) != 1:
        raise ValueError("exactly one mapped protobuf instance is required")
    protobuf_path = next(iter(protobuf))
    match = re.match(r"^libprotobuf\.so\.(\d+)(?:\.|$)", protobuf_path.name)
    if not match or int(match.group(1)) != 17:
        raise ValueError("mapped protobuf major must be exactly 17")
except (OSError, UnicodeError, ValueError) as error:
    print("invalid MID360 maps evidence: %s" % error, file=sys.stderr)
    raise SystemExit(65)

print(json.dumps({
    "plugin": str(expected),
    "protobuf": [str(protobuf_path)],
    "protobuf_major": 17,
}, sort_keys=True, separators=(",", ":")))
PY
}

if [[ "${1:-}" == "--test-mid360-maps" ]]; then
  [[ $# -eq 3 ]] || exit 64
  p450_mid360_maps_evidence "$2" "$3"
  exit $?
fi

p450_profile_plugin_maps_ready() {
  local p450_profile="$1"
  local p450_gps_plugin="$2"
  local p450_external_gps_plugin="$3"
  local p450_groundtruth_plugin="$4"
  local p450_external_groundtruth_plugin="$5"
  local p450_livox_plugin="$6"
  local p450_maps_file="$7"
  local p450_mid360_evidence="$8"
  local p450_mid360_evidence_parent="${p450_mid360_evidence%/*}"
  local p450_mid360_evidence_parent_canonical=""
  local p450_mid360_evidence_tmp=""
  local p450_mid360_maps_status=0

  case "$p450_profile" in
    d435|mid360) ;;
    *)
      echo "plugin-map profile must be exactly d435 or mid360" >&2
      return 64
      ;;
  esac
  p450_plugin_maps_is_exact \
    "$p450_gps_plugin" "$p450_external_gps_plugin" \
    "$p450_maps_file" || return
  p450_plugin_maps_is_exact \
    "$p450_groundtruth_plugin" "$p450_external_groundtruth_plugin" \
    "$p450_maps_file" || return

  if [[ "$p450_profile" == "mid360" ]]; then
    if [[ "$p450_mid360_evidence" != /* || \
        -L "$p450_mid360_evidence" || \
        ( -e "$p450_mid360_evidence" && \
          ! -f "$p450_mid360_evidence" ) ]] || \
        ! p450_mid360_evidence_parent_canonical="$({
          /usr/bin/realpath -e -- "$p450_mid360_evidence_parent"
        } 2>/dev/null)" || \
        [[ "$p450_mid360_evidence_parent_canonical" != \
           "$p450_mid360_evidence_parent" || \
           ! -d "$p450_mid360_evidence_parent" || \
           ! -w "$p450_mid360_evidence_parent" ]]; then
      echo "MID360 maps evidence target must be a regular file in a canonical writable directory" >&2
      return 66
    fi
    if ! p450_mid360_evidence_tmp="$({
      /usr/bin/mktemp --tmpdir="$p450_mid360_evidence_parent" \
        ".${p450_mid360_evidence##*/}.tmp.XXXXXX"
    })"; then
      echo "could not allocate temporary MID360 maps evidence" >&2
      return 1
    fi
    if p450_mid360_maps_evidence \
        "$p450_livox_plugin" "$p450_maps_file" \
        >"$p450_mid360_evidence_tmp"; then
      :
    else
      p450_mid360_maps_status="$?"
      /usr/bin/rm -f -- "$p450_mid360_evidence_tmp"
      return "$p450_mid360_maps_status"
    fi
    if ! /usr/bin/mv -fT -- "$p450_mid360_evidence_tmp" \
        "$p450_mid360_evidence"; then
      /usr/bin/rm -f -- "$p450_mid360_evidence_tmp"
      return 1
    fi
    if [[ ! -f "$p450_mid360_evidence" || \
        -L "$p450_mid360_evidence" ]]; then
      echo "MID360 maps evidence publication did not produce a regular file" >&2
      return 1
    fi
  elif /usr/bin/grep -Eq \
      '/liblivox_laser_gazebo_plugins\.so([[:space:]]|$)' \
      "$p450_maps_file"; then
    echo "D435 profile unexpectedly loaded the Livox plugin" >&2
    return 1
  fi
}

if [[ "${1:-}" == "--test-plugin-map-readiness" ]]; then
  [[ $# -eq 9 ]] || exit 64
  if p450_profile_plugin_maps_ready \
      "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_refresh_maps_snapshot() {
  local p450_maps_source="$1"
  local p450_maps_target="$2"
  local p450_maps_parent="${p450_maps_target%/*}"
  local p450_maps_tmp=""

  if [[ "$p450_maps_source" != /* || ! -f "$p450_maps_source" || \
      ! -r "$p450_maps_source" ]]; then
    echo "maps source must be an absolute readable regular file" >&2
    return 66
  fi
  if [[ "$p450_maps_target" != /* || \
      "${p450_maps_target##*/}" != "gzserver.maps" || \
      ! -d "$p450_maps_parent" || ! -w "$p450_maps_parent" || \
      -L "$p450_maps_target" || \
      ( -e "$p450_maps_target" && ! -f "$p450_maps_target" ) ]]; then
    echo "maps target must be a non-symlink gzserver.maps in a writable directory" >&2
    return 66
  fi
  if ! p450_maps_tmp="$({
    /usr/bin/mktemp --tmpdir="$p450_maps_parent" \
      '.gzserver.maps.tmp.XXXXXX'
  })"; then
    echo "could not allocate a temporary maps snapshot" >&2
    return 1
  fi
  if ! /usr/bin/cp -- "$p450_maps_source" "$p450_maps_tmp"; then
    /usr/bin/rm -f -- "$p450_maps_tmp"
    return 1
  fi
  if ! /usr/bin/mv -fT -- "$p450_maps_tmp" "$p450_maps_target"; then
    /usr/bin/rm -f -- "$p450_maps_tmp"
    return 1
  fi
  if [[ ! -f "$p450_maps_target" || -L "$p450_maps_target" ]]; then
    echo "maps snapshot publication did not produce a regular file" >&2
    return 1
  fi
}

if [[ "${1:-}" == "--test-refresh-maps-snapshot" ]]; then
  [[ $# -eq 3 ]] || exit 64
  if p450_refresh_maps_snapshot "$2" "$3"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_mid360_topic_evidence() {
  local p450_capture="$1"
  /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$p450_capture" <<'PY'
import json
import sys
from pathlib import Path

try:
    capture = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    if not isinstance(capture, dict):
        raise ValueError("capture must be an object")
    if capture.get("topic") != "/uav1/livox/lidar":
        raise ValueError("unexpected Livox topic")
    if capture.get("type") != "prometheus_msgs/LivoxCustomMsg":
        raise ValueError("unexpected Livox message type")
    publishers = capture.get("publishers")
    if (not isinstance(publishers, list) or len(publishers) != 1 or
            not isinstance(publishers[0], str) or not publishers[0]):
        raise ValueError("Livox topic must have exactly one publisher")
    samples = capture.get("samples")
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError("at least three Livox samples are required")
    previous_stamp = -1
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError("sample %d must be an object" % index)
        if sample.get("frame_id") != "uav1/lidar_link":
            raise ValueError("sample %d has an unexpected frame" % index)
        stamp = sample.get("stamp")
        if not isinstance(stamp, dict):
            raise ValueError("sample %d stamp must be an object" % index)
        secs = stamp.get("secs")
        nsecs = stamp.get("nsecs")
        if (type(secs) is not int or type(nsecs) is not int or secs < 0 or
                nsecs < 0 or nsecs >= 1000000000):
            raise ValueError("sample %d has an invalid stamp" % index)
        stamp_ns = secs * 1000000000 + nsecs
        if stamp_ns <= 0 or stamp_ns <= previous_stamp:
            raise ValueError("Livox stamps must be nonzero and increasing")
        previous_stamp = stamp_ns
        point_num = sample.get("point_num")
        points = sample.get("points")
        if (type(point_num) is not int or point_num <= 0 or
                not isinstance(points, list) or point_num != len(points)):
            raise ValueError("sample %d point count is invalid" % index)
except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
    print("invalid MID360 topic evidence: %s" % error, file=sys.stderr)
    raise SystemExit(65)

print(json.dumps(capture, sort_keys=True, separators=(",", ":")))
PY
}

if [[ "${1:-}" == "--test-mid360-topic" ]]; then
  [[ $# -eq 2 ]] || exit 64
  p450_mid360_topic_evidence "$2"
  exit $?
fi

p450_mid360_asset_evidence() {
  local p450_manifest="$1"
  local p450_install_share="$2"
  local p450_manifest_canonical=""
  local p450_asset_validator=""
  if ! p450_manifest_canonical="$(
    /usr/bin/realpath -e -- "$p450_manifest"
  )"; then
    echo "MID360 asset manifest is unavailable" >&2
    return 66
  fi
  p450_asset_validator="${p450_manifest_canonical%/config/mid360_assets.json}/tools/mid360_assets.py"
  if [[ "$p450_asset_validator" == "$p450_manifest_canonical" || \
      ! -f "$p450_asset_validator" ]]; then
    echo "MID360 asset validator is unavailable" >&2
    return 66
  fi
  if ! /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
      /usr/bin/python3 -I -B "$p450_asset_validator" \
        --config "$p450_manifest_canonical" \
        --asset-root "$p450_install_share" >/dev/null; then
    return 65
  fi
  /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$p450_manifest" "$p450_install_share" <<'PY'
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

expected_targets = {
    "models/MID360/model.config",
    "models/MID360/MID360.sdf",
    "models/MID360/meshes/MID360.dae",
    "models/MID360/scan_mode/mid360.csv",
}
try:
    manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    install_share = Path(sys.argv[2]).resolve(strict=True)
    if not install_share.is_dir() or install_share.name != "sim_platform_assets":
        raise ValueError("install share must be sim_platform_assets")
    entries = manifest.get("files") if isinstance(manifest, dict) else None
    if not isinstance(entries, list) or len(entries) != 4:
        raise ValueError("asset ledger must contain exactly four files")
    if {entry.get("target") for entry in entries} != expected_targets:
        raise ValueError("asset ledger target set is not exact")
    evidence = []
    paths = {}
    for entry in entries:
        target = entry.get("target")
        digest = entry.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError("invalid asset digest for %s" % target)
        lexical = install_share / target
        if lexical.is_symlink():
            raise ValueError("installed asset must not be a symlink: %s" % target)
        resolved = lexical.resolve(strict=True)
        resolved.relative_to(install_share)
        if not resolved.is_file():
            raise ValueError("installed asset is not a regular file: %s" % target)
        observed = hashlib.sha256(resolved.read_bytes()).hexdigest()
        if observed != digest:
            raise ValueError("asset hash mismatch: %s" % target)
        paths[target] = resolved
        evidence.append({"path": str(resolved), "sha256": observed,
                         "target": target})
    sdf = ET.parse(str(paths["models/MID360/MID360.sdf"])).getroot()
    uris = [node.text.strip() for node in sdf.findall(".//uri")
            if node.text and node.text.strip()]
    if not uris or set(uris) != {"model://MID360/meshes/MID360.dae"}:
        raise ValueError("MID360 SDF mesh URI closure is not exact")
    dae = ET.parse(str(paths["models/MID360/meshes/MID360.dae"])).getroot()
    for element in dae.iter():
        for attribute in ("url", "source"):
            reference = element.get(attribute)
            if reference and not reference.startswith("#"):
                raise ValueError("external DAE reference is forbidden: %s" % reference)
except (OSError, UnicodeError, json.JSONDecodeError, ET.ParseError,
        KeyError, TypeError, ValueError) as error:
    print("invalid MID360 asset evidence: %s" % error, file=sys.stderr)
    raise SystemExit(65)

print(json.dumps({
    "csv": str(paths["models/MID360/scan_mode/mid360.csv"]),
    "dae": str(paths["models/MID360/meshes/MID360.dae"]),
    "files": evidence,
    "sdf": str(paths["models/MID360/MID360.sdf"]),
    "uris": uris,
}, sort_keys=True, separators=(",", ":")))
PY
}

if [[ "${1:-}" == "--test-mid360-assets" ]]; then
  [[ $# -eq 3 ]] || exit 64
  p450_mid360_asset_evidence "$2" "$3"
  exit $?
fi

p450_mid360_csv_log_evidence() {
  local p450_expected_csv="$1"
  local p450_launch_log="$2"
  /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$p450_expected_csv" "$p450_launch_log" <<'PY'
import json
import re
import sys
from pathlib import Path

try:
    expected = Path(sys.argv[1]).resolve(strict=True)
    launch_log = Path(sys.argv[2])
    if not expected.is_file() or not launch_log.is_file():
        raise ValueError("CSV and launch log must be regular files")
    matches = []
    ansi_csi = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
    marker = "load csv file name:"
    marker_count = 0
    pattern = re.compile(re.escape(marker) + r"(/[^\r\n]+)$")
    for line in launch_log.read_text(
            encoding="utf-8", errors="surrogateescape").splitlines():
        line = ansi_csi.sub("", line)
        occurrences = line.count(marker)
        if occurrences == 0:
            continue
        marker_count += occurrences
        if occurrences != 1:
            raise ValueError("CSV log line must contain exactly one marker")
        match = pattern.search(line)
        if not match:
            raise ValueError("CSV log marker does not contain an absolute path")
        matches.append(Path(match.group(1)).resolve(strict=True))
    if marker_count != 1 or matches != [expected]:
        raise ValueError("expected one exact resolved CSV path, got %s" %
                         [str(path) for path in matches])
except (OSError, UnicodeError, ValueError) as error:
    print("invalid MID360 CSV log evidence: %s" % error, file=sys.stderr)
    raise SystemExit(65)
print(json.dumps({"csv": str(expected)}, sort_keys=True,
                 separators=(",", ":")))
PY
}

if [[ "${1:-}" == "--test-mid360-csv-log" ]]; then
  [[ $# -eq 3 ]] || exit 64
  p450_mid360_csv_log_evidence "$2" "$3"
  exit $?
fi

p450_mid360_tf_evidence() {
  local p450_contract="$1"
  local p450_capture="$2"
  local p450_authorities="$3"
  /usr/bin/env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - \
      "$p450_contract" "$p450_capture" "$p450_authorities" <<'PY'
import json
import math
import sys
from pathlib import Path

import yaml

try:
    contract_path = Path(sys.argv[1]).resolve(strict=True)
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    capture = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    authorities = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
    composite = contract["composite"]
    tolerance = contract["runtime_tolerance"]
    if capture.get("parent") != composite["parent"]:
        raise ValueError("unexpected MID360 TF parent")
    if capture.get("child") != composite["child"]:
        raise ValueError("unexpected MID360 TF child")
    for key, contract_key, tolerance_key in (
            ("translation", "translation_m", "translation_m"),
            ("rotation_xyzw", "rotation_xyzw", "rotation_xyzw")):
        observed = capture.get(key)
        expected = composite[contract_key]
        allowed = tolerance[tolerance_key]
        expected_length = 3 if key == "translation" else 4
        if (type(allowed) not in (int, float) or
                not math.isfinite(allowed) or allowed <= 0 or
                not isinstance(expected, list) or
                len(expected) != expected_length or
                any(type(value) not in (int, float) or
                    not math.isfinite(value) for value in expected) or
                not isinstance(observed, list) or
                len(observed) != expected_length or
                any(type(value) not in (int, float) or
                    not math.isfinite(value) for value in observed) or
                any(abs(float(left) - float(right)) > float(allowed)
                    for left, right in zip(observed, expected))):
            raise ValueError("MID360 %s exceeds installed tolerance" % key)
    if not isinstance(authorities, dict):
        raise ValueError("TF authorities must be an object")
    for child, callers in authorities.items():
        if (not isinstance(child, str) or not child or
                not isinstance(callers, list) or len(callers) != 1 or
                not isinstance(callers[0], str) or not callers[0]):
            raise ValueError("every global TF child must have one authority")
    lidar_authority = authorities.get(composite["child"])
    if lidar_authority != [composite["authority"]]:
        raise ValueError("unexpected MID360 TF authority")
except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError,
        ValueError, yaml.YAMLError) as error:
    print("invalid MID360 TF evidence: %s" % error, file=sys.stderr)
    raise SystemExit(65)

print(json.dumps({
    "authority": composite["authority"],
    "contract": str(contract_path),
    "tolerance": tolerance,
    "transform": capture,
}, sort_keys=True, separators=(",", ":")))
PY
}

if [[ "${1:-}" == "--test-mid360-tf" ]]; then
  [[ $# -eq 4 ]] || exit 64
  p450_mid360_tf_evidence "$2" "$3" "$4"
  exit $?
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
    "uav1/local_origin": {"/uav1/p450_tf_world_local_origin"},
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
  [[ $# -eq 12 ]] || return 64
  /usr/bin/env -i \
    PATH=/usr/bin:/bin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B - "$@" <<'PY'
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path

import yaml


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
sensor_profile = sys.argv[8]
gps_plugin = Path(sys.argv[9]).resolve(strict=True)
groundtruth_plugin = Path(sys.argv[10]).resolve(strict=True)
common_path = Path(sys.argv[11])
sensor_path = Path(sys.argv[12])

if verdict not in {"PASS", "FAIL"}:
    raise ValueError("verdict must be PASS or FAIL")
if (verdict == "PASS") != (script_status == 0):
    raise ValueError("verdict and script status disagree")
if shutdown_escalated_raw not in {"0", "1"}:
    raise ValueError("shutdown_escalated must be 0 or 1")
if (verdict == "PASS" and
        (launcher_status != 0 or shutdown_escalated_raw != "0")):
    raise ValueError(
        "PASS requires launcher status zero and no shutdown escalation")
if not run_id or not px4_workdir:
    raise ValueError("run identity fields must be non-empty")
if not gps_plugin.is_file() or not groundtruth_plugin.is_file():
    raise ValueError("plugin paths must name regular files")
if sensor_profile not in {"d435", "mid360"}:
    raise ValueError("sensor profile must be d435 or mid360")
if result_path.exists() or not result_path.parent.is_dir():
    raise ValueError("result path must be new and have an existing parent")


def load_evidence(path, required):
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("evidence must be an object")
        return value
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        if required:
            raise ValueError("required PASS evidence is invalid: %s" % path)
        return None


common = load_evidence(common_path, verdict == "PASS")
sensor = load_evidence(sensor_path, verdict == "PASS")
if verdict == "PASS":
    expected_checks = {
        "gazebo_node", "gzserver_process", "mavros_node", "mavros_state",
        "model", "prometheus_state", "px4_process", "uav_controller_node",
    }
    expected_authorities = {
        "uav1/local_origin": ["/uav1/p450_tf_world_local_origin"],
        "uav1/base_link": ["/uav_control_main_1"],
        "uav1/camera_link": ["/uav_control_main_1"],
        "uav1/camera_depth_frame": ["/uav1/p450_tf_camera_depth"],
        "uav1/camera_ired1_frame": ["/uav1/p450_tf_camera_ired1"],
        "uav1/camera_ired2_frame": ["/uav1/p450_tf_camera_ired2"],
        "uav1/camera_imu_link": ["/uav1/p450_tf_camera_imu"],
        "uav1/d435i_link": ["/uav1/p450_tf_camera_d435i"],
        "uav1/camera_color_optical_frame": [
            "/uav1/p450_tf_camera_color_optical"],
        "uav1/camera_depth_optical_frame": [
            "/uav1/p450_tf_depth_optical"],
    }
    authorities = common.get("tf_authorities", {})
    if (set(common) != {"checks", "model", "tf_authorities"} or
            set(common.get("checks", {})) != expected_checks or
            any(value is not True for value in common["checks"].values()) or
            common.get("model") != {"count": 1, "name": "p450_D435i_0"} or
            not isinstance(authorities, dict) or
            any(authorities.get(child) != callers
                for child, callers in expected_authorities.items()) or
            any(not isinstance(callers, list) or len(callers) != 1
                for callers in authorities.values()) or
            (sensor_profile == "mid360" and
             authorities.get("uav1/lidar_link") !=
             ["/uav_control_main_1"])):
        raise ValueError("common PASS evidence is incomplete")
    if sensor_profile == "d435":
        expected_topics = [
            "/uav1/camera/color/image_raw",
            "/uav1/camera/color/camera_info",
            "/uav1/camera/depth/image_raw",
            "/uav1/camera/depth/camera_info",
            "/uav1/camera/imu",
        ]
        if (set(sensor) != {"livox_absent", "profile", "tf_current",
                           "topic_gates"} or
                sensor.get("profile") != "d435" or
                sensor.get("livox_absent") is not True or
                sensor.get("tf_current") is not True or
                sensor.get("topic_gates") != expected_topics):
            raise ValueError("D435 PASS evidence is incomplete")
    else:
        if (set(sensor) != {"assets", "csv_log", "maps", "profile", "tf",
                           "topic"} or sensor.get("profile") != "mid360"):
            raise ValueError("MID360 PASS evidence is incomplete")

        def canonical_file(raw, label):
            if not isinstance(raw, str) or not raw.startswith("/"):
                raise ValueError("%s path is not absolute" % label)
            lexical = Path(raw)
            resolved = lexical.resolve(strict=True)
            if (str(resolved) != raw or lexical.is_symlink() or
                    not resolved.is_file()):
                raise ValueError("%s path is not canonical" % label)
            return resolved

        tf = sensor["tf"]
        if (not isinstance(tf, dict) or
                set(tf) != {"authority", "contract", "tolerance",
                            "transform"} or
                tf.get("authority") != "/uav_control_main_1"):
            raise ValueError("MID360 TF evidence schema is incomplete")
        contract_path = canonical_file(tf.get("contract"), "TF contract")
        if contract_path.name != "p450_mid360_tf_contract.yaml":
            raise ValueError("unexpected MID360 TF contract")
        install_prefix = contract_path.parents[3]
        repository_root = contract_path.parents[5]
        expected_contract = (install_prefix / "share" /
            "sim_platform_bringup/config/p450_mid360_tf_contract.yaml")
        if contract_path != expected_contract:
            raise ValueError("TF contract is outside the canonical install")
        contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
        composite = contract.get("composite", {})
        expected_tolerance = contract.get("runtime_tolerance")
        tolerance = tf.get("tolerance")
        transform = tf.get("transform")
        if (not isinstance(tolerance, dict) or
                set(tolerance) != {"translation_m", "rotation_xyzw"} or
                not isinstance(transform, dict) or
                set(transform) != {"parent", "child", "translation",
                                   "rotation_xyzw"}):
            raise ValueError("MID360 TF numeric evidence schema is incomplete")
        for key in ("translation_m", "rotation_xyzw"):
            value = tolerance.get(key)
            if (type(value) not in (int, float) or
                    not math.isfinite(value) or value <= 0):
                raise ValueError("MID360 TF tolerance is invalid")
        for observed_key, expected_key, length in (
                ("translation", "translation_m", 3),
                ("rotation_xyzw", "rotation_xyzw", 4)):
            observed = transform.get(observed_key)
            expected = composite.get(expected_key)
            if (not isinstance(observed, list) or len(observed) != length or
                    not isinstance(expected, list) or len(expected) != length or
                    any(type(item) not in (int, float) or
                        not math.isfinite(item)
                        for item in observed + expected)):
                raise ValueError("MID360 TF transform is invalid")
        if (tolerance != expected_tolerance or
                transform.get("parent") != composite.get("parent") or
                transform.get("child") != composite.get("child") or
                composite.get("authority") != tf["authority"]):
            raise ValueError("MID360 TF evidence disagrees with its contract")
        for observed_key, expected_key, tolerance_key in (
                ("translation", "translation_m", "translation_m"),
                ("rotation_xyzw", "rotation_xyzw", "rotation_xyzw")):
            if any(abs(float(observed) - float(expected)) >
                   float(tolerance[tolerance_key])
                   for observed, expected in zip(
                       transform[observed_key], composite[expected_key])):
                raise ValueError(
                    "MID360 TF evidence exceeds its contract tolerance")

        maps = sensor["maps"]
        expected_plugin = (install_prefix / "lib" /
                           "liblivox_laser_gazebo_plugins.so")
        if (not isinstance(maps, dict) or
                set(maps) != {"plugin", "protobuf", "protobuf_major"} or
                maps.get("protobuf_major") != 17 or
                canonical_file(maps.get("plugin"), "Livox plugin") !=
                expected_plugin):
            raise ValueError("MID360 maps evidence is incomplete")
        protobuf = maps.get("protobuf")
        if not isinstance(protobuf, list) or len(protobuf) != 1:
            raise ValueError("MID360 protobuf evidence is incomplete")
        protobuf_path = canonical_file(protobuf[0], "protobuf")
        match = re.match(r"^libprotobuf\.so\.(\d+)(?:\.|$)",
                         protobuf_path.name)
        if not match or int(match.group(1)) != 17:
            raise ValueError("MID360 protobuf major is not 17")

        assets = sensor["assets"]
        asset_root = install_prefix / "share/sim_platform_assets"
        manifest_path = repository_root / "config/mid360_assets.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        ledger = manifest.get("files")
        records = assets.get("files") if isinstance(assets, dict) else None
        if (not isinstance(assets, dict) or
                set(assets) != {"csv", "dae", "files", "sdf", "uris"} or
                not isinstance(ledger, list) or len(ledger) != 4 or
                not isinstance(records, list) or len(records) != 4):
            raise ValueError("MID360 asset evidence schema is incomplete")
        expected_entries = {entry["target"]: entry["sha256"]
                            for entry in ledger}
        observed_entries = {}
        for record in records:
            if not isinstance(record, dict) or set(record) != {
                    "path", "sha256", "target"}:
                raise ValueError("MID360 asset record schema is incomplete")
            target = record.get("target")
            if target in observed_entries or target not in expected_entries:
                raise ValueError("MID360 asset target set is not exact")
            expected_path = (asset_root / target).resolve(strict=True)
            observed_path = canonical_file(record.get("path"), "asset")
            digest = hashlib.sha256(observed_path.read_bytes()).hexdigest()
            if (observed_path != expected_path or
                    record.get("sha256") != expected_entries[target] or
                    digest != expected_entries[target]):
                raise ValueError("MID360 asset path or digest disagrees")
            observed_entries[target] = observed_path
        if set(observed_entries) != set(expected_entries):
            raise ValueError("MID360 asset target set is not exact")
        expected_csv = observed_entries[
            "models/MID360/scan_mode/mid360.csv"]
        expected_dae = observed_entries["models/MID360/meshes/MID360.dae"]
        expected_sdf = observed_entries["models/MID360/MID360.sdf"]
        if (canonical_file(assets.get("csv"), "asset CSV") != expected_csv or
                canonical_file(assets.get("dae"), "asset DAE") != expected_dae or
                canonical_file(assets.get("sdf"), "asset SDF") != expected_sdf or
                assets.get("uris") !=
                ["model://MID360/meshes/MID360.dae"] or
                sensor.get("csv_log") != {"csv": str(expected_csv)}):
            raise ValueError("MID360 asset cross-links are inconsistent")

        topic = sensor["topic"]
        if (not isinstance(topic, dict) or
                set(topic) != {"topic", "type", "publishers", "samples"} or
                topic.get("topic") != "/uav1/livox/lidar" or
                topic.get("type") != "prometheus_msgs/LivoxCustomMsg" or
                not isinstance(topic.get("publishers"), list) or
                len(topic["publishers"]) != 1 or
                not isinstance(topic["publishers"][0], str) or
                not topic["publishers"][0] or
                not isinstance(topic.get("samples"), list) or
                len(topic["samples"]) < 3):
            raise ValueError("MID360 topic evidence schema is incomplete")
        previous_stamp = -1
        for sample in topic["samples"]:
            if (not isinstance(sample, dict) or
                    set(sample) != {"stamp", "frame_id", "point_num", "points"} or
                    sample.get("frame_id") != "uav1/lidar_link"):
                raise ValueError("MID360 topic sample schema is invalid")
            stamp = sample.get("stamp")
            if not isinstance(stamp, dict) or set(stamp) != {"secs", "nsecs"}:
                raise ValueError("MID360 topic stamp schema is invalid")
            secs, nsecs = stamp.get("secs"), stamp.get("nsecs")
            stamp_ns = (secs * 1000000000 + nsecs
                        if type(secs) is int and type(nsecs) is int else -1)
            points, point_num = sample.get("points"), sample.get("point_num")
            if (type(secs) is not int or type(nsecs) is not int or secs < 0 or
                    nsecs < 0 or nsecs >= 1000000000 or
                    stamp_ns <= 0 or stamp_ns <= previous_stamp or
                    type(point_num) is not int or point_num <= 0 or
                    not isinstance(points, list) or point_num != len(points)):
                raise ValueError("MID360 topic sample values are invalid")
            previous_stamp = stamp_ns

payload = {
    "schema_version": 2,
    "verdict": verdict,
    "script_status": script_status,
    "launcher_status": launcher_status,
    "shutdown_escalated": shutdown_escalated_raw == "1",
    "run_id": run_id,
    "px4_workdir": px4_workdir,
    "sensor_profile": sensor_profile,
    "plugins": {
        "gps": str(gps_plugin),
        "groundtruth": str(groundtruth_plugin),
    },
    "evidence": {"common": common, "sensor": sensor},
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

p450_result_fallback_written=0
p450_write_result_with_fallback() {
  [[ $# -eq 12 ]] || return 64
  local p450_initial_status=0
  p450_result_fallback_written=0
  if p450_write_result_manifest "$@"; then
    return 0
  else
    p450_initial_status="$?"
  fi
  if [[ "$2" != "PASS" || -e "$1" ]]; then
    return "$p450_initial_status"
  fi
  if p450_write_result_manifest \
      "$1" FAIL 1 "$4" "$5" "$6" "$7" "$8" "$9" \
      "${10}" "${11}" "${12}"; then
    p450_result_fallback_written=1
  fi
  return "$p450_initial_status"
}

if [[ "${1:-}" == "--test-result-manifest" ]]; then
  [[ $# -eq 13 ]] || exit 64
  if p450_write_result_manifest \
      "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" \
      "${11}" "${12}" "${13}"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

if [[ "${1:-}" == "--test-result-fallback" ]]; then
  [[ $# -eq 13 ]] || exit 64
  if p450_write_result_with_fallback \
      "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" "${10}" \
      "${11}" "${12}" "${13}"; then
    exit 0
  else
    p450_probe_status="$?"
    exit "$p450_probe_status"
  fi
fi

p450_test_inner_boundary=0
if [[ "${1:-}" == "--test-inner-boundary" ]]; then
  [[ $# -eq 1 && "${P450_SMOKE_INNER:-0}" == "1" ]] || exit 64
  p450_test_inner_boundary=1
elif [[ $# -ne 0 ]]; then
  echo "P450 standalone smoke accepts no runtime arguments" >&2
  exit 64
fi

p450_script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
p450_script_dir="${p450_script_path%/*}"
p450_repo_root="${p450_script_dir%/*}"
p450_environment_wrapper="$p450_repo_root/scripts/with_p450_env.bash"
p450_current_environment_validator="$p450_repo_root/scripts/validate_p450_current_environment.bash"

if [[ "${P450_SMOKE_INNER:-0}" == "1" ]]; then
  [[ -x "$p450_current_environment_validator" ]] || \
    p450_die "current environment validator is missing or not executable"
  "$p450_current_environment_validator" \
    "$p450_repo_root" "${P450_PX4_ROOT:-}"
  if (( p450_test_inner_boundary )); then
    exit 0
  fi
fi

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

if [[ "${P450_SMOKE_INNER:-0}" != "1" ]]; then
  [[ -x "$p450_environment_wrapper" ]] || \
    p450_die "runtime environment wrapper is missing or not executable"
  export P450_GAZEBO_DISPLAY="$p450_render_display"
  export P450_GAZEBO_XAUTHORITY="$p450_render_xauthority"
  exec "$p450_environment_wrapper" /usr/bin/env \
    P450_SMOKE_INNER=1 \
    P450_SENSOR_PROFILE="$p450_sensor_profile" \
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
[[ "${P450_SENSOR_PROFILE:-}" == "$p450_sensor_profile" ]] || \
  p450_die "inner P450_SENSOR_PROFILE changed across the environment wrapper"
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
p450_log_parent="$p450_repo_root/$p450_profile_log_root"
if [[ ! -e "$p450_log_parent" && ! -L "$p450_log_parent" ]]; then
  /usr/bin/mkdir --mode=700 -- "$p450_log_parent" || \
    p450_die "could not create the profile log directory"
fi
if p450_log_dir="$({
  p450_create_private_run_directory "$p450_log_parent" "$p450_run_id"
})"; then
  :
else
  p450_run_directory_status="$?"
  p450_die "could not create a private run directory ($p450_run_directory_status)"
fi
p450_launch_log="$p450_log_dir/roslaunch.log"
p450_common_evidence="$p450_log_dir/common-evidence.json"
p450_sensor_evidence="$p450_log_dir/sensor-evidence.json"
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
  if ! p450_write_result_with_fallback \
      "$p450_log_dir/result.json" \
      "$p450_verdict" \
      "$p450_status" \
      "$p450_launch_status" \
      "$p450_shutdown_escalated" \
      "$p450_run_id" \
      "$p450_px4_workdir" \
      "$p450_sensor_profile" \
      "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_gps_plugin.so" \
      "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_groundtruth_plugin.so" \
      "$p450_common_evidence" \
      "$p450_sensor_evidence"; then
    p450_status=1
    if (( p450_result_fallback_written )); then
      echo "P450 standalone smoke: PASS evidence was rejected; wrote FAIL result.json" >&2
    else
      echo "P450 standalone smoke: could not write result.json" >&2
    fi
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
    "${p450_profile_launch_args[@]}" \
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

p450_try_capture_complete_plugin_maps() {
  local p450_gzserver_pid
  if ! p450_gzserver_pid="$(p450_find_unique_gzserver_pid)"; then
    return 1
  fi
  if ! p450_refresh_maps_snapshot "/proc/$p450_gzserver_pid/maps" \
      "$p450_log_dir/gzserver.maps"; then
    echo "could not capture gzserver plugin mappings" >&2
    return 1
  fi
  p450_profile_plugin_maps_ready \
    "$p450_sensor_profile" \
    "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_gps_plugin.so" \
    "$P450_PX4_ROOT/build/amovlab_sitl_default/build_gazebo/libgazebo_gps_plugin.so" \
    "$p450_repo_root/install/p450-runtime-overlays/lib/libgazebo_groundtruth_plugin.so" \
    "$P450_PX4_ROOT/build/amovlab_sitl_default/build_gazebo/libgazebo_groundtruth_plugin.so" \
    "$p450_repo_root/install/p450-clean/lib/liblivox_laser_gazebo_plugins.so" \
    "$p450_log_dir/gzserver.maps" \
    "$p450_log_dir/mid360-maps.json"
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
p450_wait_until "complete Gazebo plugin mappings" \
  "$p450_log_dir/plugin-maps.wait.log" \
  p450_try_capture_complete_plugin_maps

if [[ "$p450_sensor_profile" == "mid360" ]]; then
  if ! p450_mid360_asset_evidence \
      "$p450_repo_root/config/mid360_assets.json" \
      "$p450_repo_root/install/p450-clean/share/sim_platform_assets" \
      >"$p450_log_dir/mid360-assets.json"; then
    p450_die "installed MID360 asset closure is invalid"
  fi
  p450_mid360_csv_path="$({
    /usr/bin/python3 -c \
      'import json,sys; print(json.load(open(sys.argv[1]))["csv"])' \
      "$p450_log_dir/mid360-assets.json"
  })"
  if ! p450_mid360_csv_log_evidence \
      "$p450_mid360_csv_path" "$p450_launch_log" \
      >"$p450_log_dir/mid360-csv-log.json"; then
    p450_die "Livox plugin did not log the exact installed MID360 CSV path"
  fi
fi

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
if [[ "$p450_sensor_profile" == "d435" ]] && \
    /usr/bin/timeout --signal=TERM --kill-after=1s 3s \
      /opt/ros/noetic/bin/rostopic type /uav1/livox/lidar \
      >"$p450_log_dir/livox-absence.log" 2>&1; then
  p450_die "D435 profile unexpectedly advertised the Livox topic"
fi

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

if [[ "$p450_sensor_profile" == "d435" ]]; then
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
else
  if ! /usr/bin/timeout --signal=TERM --kill-after=2s \
      "${p450_topic_seconds}s" /usr/bin/python3 - \
      "$p450_log_dir/mid360-topic-capture.json" \
      >"$p450_log_dir/mid360-topic-collect.log" 2>&1 <<'PY'
import json
import sys

import rosgraph
import rospy
from prometheus_msgs.msg import LivoxCustomMsg

topic = "/uav1/livox/lidar"
rospy.init_node("p450_smoke_mid360_topic", anonymous=True,
                disable_signals=True)
published_types = dict(rospy.get_published_topics(namespace="/"))
if published_types.get(topic) != "prometheus_msgs/LivoxCustomMsg":
    raise SystemExit("unexpected MID360 topic type: %r" %
                     published_types.get(topic))
publishers = []
for published_topic, callers in rosgraph.Master(
        rospy.get_name()).getSystemState()[0]:
    if published_topic == topic:
        publishers = sorted(callers)
if len(publishers) != 1:
    raise SystemExit("expected one MID360 publisher, got %r" % publishers)
samples = []

def collect(message):
    if len(samples) >= 3:
        return
    samples.append({
        "frame_id": message.header.frame_id,
        "point_num": message.point_num,
        "points": [None] * len(message.points),
        "stamp": {
            "nsecs": message.header.stamp.nsecs,
            "secs": message.header.stamp.secs,
        },
    })

subscriber = rospy.Subscriber(topic, LivoxCustomMsg, collect, queue_size=5)
deadline = rospy.Time.now().to_sec() + 20.0
while len(samples) < 3 and not rospy.is_shutdown():
    rospy.sleep(0.02)
    if rospy.Time.now().to_sec() > deadline:
        break
del subscriber
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump({"publishers": publishers, "samples": samples,
               "topic": topic,
               "type": "prometheus_msgs/LivoxCustomMsg"}, stream,
              sort_keys=True)
PY
  then
    /usr/bin/tail -n 100 "$p450_log_dir/mid360-topic-collect.log" >&2 || true
    p450_die "MID360 topic capture failed"
  fi
  if ! p450_mid360_topic_evidence \
      "$p450_log_dir/mid360-topic-capture.json" \
      >"$p450_log_dir/mid360-topic.json"; then
    p450_die "MID360 topic contract failed"
  fi
fi

if ! /usr/bin/timeout --signal=TERM --kill-after=2s 25s \
    /usr/bin/python3 - "$p450_sensor_profile" \
      "$p450_log_dir/mid360-tf-capture.json" \
      >"$p450_log_dir/tf-current.log" 2>&1 <<'PY'
import json
import sys
import time

import rospy
import tf2_ros
from rosgraph_msgs.msg import Clock

rospy.init_node("p450_smoke_tf_current", anonymous=True, disable_signals=True)
rospy.wait_for_message("/clock", Clock, timeout=10.0)
buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
listener = tf2_ros.TransformListener(buffer)
frames = [
    "uav1/base_link",
    "uav1/camera_link",
    "uav1/camera_depth_frame",
    "uav1/camera_ired1_frame",
    "uav1/camera_ired2_frame",
    "uav1/camera_imu_link",
    "uav1/d435i_link",
    "uav1/camera_color_optical_frame",
    "uav1/camera_depth_optical_frame",
]
if sys.argv[1] == "mid360":
    frames.append("uav1/lidar_link")
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
if sys.argv[1] == "mid360":
    transform = buffer.lookup_transform(
        "uav1/base_link", "uav1/lidar_link", rospy.Time(0),
        rospy.Duration(1.0))
    value = transform.transform
    with open(sys.argv[2], "x", encoding="utf-8") as stream:
        json.dump({
            "child": "uav1/lidar_link",
            "parent": "uav1/base_link",
            "rotation_xyzw": [value.rotation.x, value.rotation.y,
                              value.rotation.z, value.rotation.w],
            "translation": [value.translation.x, value.translation.y,
                            value.translation.z],
        }, stream, sort_keys=True)
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
if [[ "$p450_sensor_profile" == "mid360" ]]; then
  if ! p450_mid360_tf_evidence \
      "$p450_repo_root/install/p450-clean/share/sim_platform_bringup/config/p450_mid360_tf_contract.yaml" \
      "$p450_log_dir/mid360-tf-capture.json" \
      "$p450_log_dir/tf-authorities.json" \
      >"$p450_log_dir/mid360-tf.json"; then
    p450_die "MID360 installed TF contract failed"
  fi
fi

/usr/bin/python3 - "$p450_common_evidence" \
    "$p450_log_dir/tf-authorities.json" <<'PY'
import json
import sys

with open(sys.argv[2], "r", encoding="utf-8") as stream:
    authorities = json.load(stream)
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump({
        "checks": {
            "gazebo_node": True,
            "gzserver_process": True,
            "mavros_node": True,
            "mavros_state": True,
            "model": True,
            "prometheus_state": True,
            "px4_process": True,
            "uav_controller_node": True,
        },
        "model": {"count": 1, "name": "p450_D435i_0"},
        "tf_authorities": authorities,
    }, stream, sort_keys=True)
PY
if [[ "$p450_sensor_profile" == "d435" ]]; then
  /usr/bin/python3 - "$p450_sensor_evidence" <<'PY'
import json
import sys

with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump({
        "livox_absent": True,
        "profile": "d435",
        "tf_current": True,
        "topic_gates": [
            "/uav1/camera/color/image_raw",
            "/uav1/camera/color/camera_info",
            "/uav1/camera/depth/image_raw",
            "/uav1/camera/depth/camera_info",
            "/uav1/camera/imu",
        ],
    }, stream, sort_keys=True)
PY
else
  /usr/bin/python3 - \
      "$p450_sensor_evidence" \
      "$p450_log_dir/mid360-assets.json" \
      "$p450_log_dir/mid360-csv-log.json" \
      "$p450_log_dir/mid360-maps.json" \
      "$p450_log_dir/mid360-tf.json" \
      "$p450_log_dir/mid360-topic.json" <<'PY'
import json
import sys

names = ("assets", "csv_log", "maps", "tf", "topic")
payload = {"profile": "mid360"}
for name, path in zip(names, sys.argv[2:]):
    with open(path, "r", encoding="utf-8") as stream:
        payload[name] = json.load(stream)
with open(sys.argv[1], "x", encoding="utf-8") as stream:
    json.dump(payload, stream, sort_keys=True)
PY
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
