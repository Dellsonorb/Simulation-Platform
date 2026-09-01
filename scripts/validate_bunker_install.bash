#!/bin/bash

set -euo pipefail

bunker_install_fail() {
  echo "validate-bunker-install: $2" >&2
  exit "$1"
}

bunker_validator_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")" || \
  bunker_install_fail 65 "cannot resolve validator path"
bunker_script_dir="${bunker_validator_path%/*}"
bunker_expected_repo="${bunker_script_dir%/*}"
bunker_expected_run_root="$bunker_expected_repo/logs/bunker_standalone"
bunker_expected_install="$bunker_expected_repo/install/p450-clean"
bunker_plugin_planar="/opt/ros/noetic/lib/libgazebo_ros_planar_move.so"
bunker_plugin_laser="/opt/ros/noetic/lib/libgazebo_ros_laser.so"
bunker_plugin_ray="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so"

bunker_require_variable() {
  [[ $# -eq 1 && -n "${!1+x}" ]] || return 1
}

bunker_validate_environment() {
  local -a required=(
    BUNKER_REPO_ROOT BUNKER_RUN_DIR TMPDIR ROS_HOME ROS_LOG_DIR
    GAZEBO_LOG_PATH IGN_FUEL_CACHE_PATH XDG_CACHE_HOME XDG_CONFIG_HOME
    XDG_DATA_HOME ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH
    LD_LIBRARY_PATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH
    GAZEBO_RESOURCE_PATH PKG_CONFIG_PATH OGRE_RESOURCE_PATH ROS_ETC_DIR
    ROS_ROOT ROSLISP_PACKAGE_DIRECTORIES GAZEBO_MODEL_DATABASE_URI
    PATH LANG LC_ALL
  )
  local variable
  for variable in "${required[@]}"; do
    bunker_require_variable "$variable" || return 1
  done
  [[ "$BUNKER_REPO_ROOT" == "$bunker_expected_repo" && \
     "$BUNKER_REPO_ROOT" == \
       "$(/usr/bin/realpath -e -- "$BUNKER_REPO_ROOT")" ]] || return 1
  [[ "$BUNKER_RUN_DIR" == /* && -d "$BUNKER_RUN_DIR" && \
     ! -L "$BUNKER_RUN_DIR" && \
     "$BUNKER_RUN_DIR" == \
       "$(/usr/bin/realpath -e -- "$BUNKER_RUN_DIR")" && \
     "$(/usr/bin/stat -c '%a' -- "$BUNKER_RUN_DIR")" == "700" ]] || \
    return 1
  case "$BUNKER_RUN_DIR" in
    "$bunker_expected_run_root"/*) ;;
    *) return 1 ;;
  esac
  local -a state_names=(
    TMPDIR ROS_HOME ROS_LOG_DIR GAZEBO_LOG_PATH IGN_FUEL_CACHE_PATH
    XDG_CACHE_HOME XDG_CONFIG_HOME XDG_DATA_HOME
  )
  local -a state_suffixes=(
    tmp ros-home ros-log gazebo-log ign-fuel-cache xdg-cache xdg-config xdg-data
  )
  local index actual expected
  for index in 0 1 2 3 4 5 6 7; do
    variable="${state_names[$index]}"
    actual="${!variable}"
    expected="$BUNKER_RUN_DIR/${state_suffixes[$index]}"
    [[ "$actual" == "$expected" && -d "$actual" && ! -L "$actual" && \
       "$actual" == "$(/usr/bin/realpath -e -- "$actual")" && \
       "$(/usr/bin/stat -c '%a' -- "$actual")" == "700" ]] || return 1
  done
  local install="$bunker_expected_install"
  [[ "$ROS_PACKAGE_PATH" == "$install/share:/opt/ros/noetic/share" ]] || \
    return 1
  [[ "$CMAKE_PREFIX_PATH" == "$install:/opt/ros/noetic" ]] || return 1
  [[ "$PYTHONPATH" == \
     "$install/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages" ]] || \
    return 1
  [[ "$LD_LIBRARY_PATH" == \
     "$install/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" ]] || \
    return 1
  [[ "$GAZEBO_PLUGIN_PATH" == \
     "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib" ]] || \
    return 1
  [[ "$GAZEBO_MODEL_PATH" == "/usr/share/gazebo-11/models" ]] || return 1
  [[ "$GAZEBO_RESOURCE_PATH" == "/usr/share/gazebo-11" ]] || return 1
  [[ "$PKG_CONFIG_PATH" == \
     "$install/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig" ]] || \
    return 1
  [[ "$OGRE_RESOURCE_PATH" == "/usr/lib/x86_64-linux-gnu/OGRE-1.9.0" ]] || \
    return 1
  [[ "$ROS_ETC_DIR" == "/opt/ros/noetic/etc/ros" && \
     "$ROS_ROOT" == "/opt/ros/noetic/share/ros" && \
     -z "$ROSLISP_PACKAGE_DIRECTORIES" && \
     -z "$GAZEBO_MODEL_DATABASE_URI" && \
     "$PATH" == "/opt/ros/noetic/bin:/usr/bin:/bin" && \
     "$LANG" == "C.UTF-8" && "$LC_ALL" == "C.UTF-8" ]] || return 1
  for variable in ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH \
      LD_LIBRARY_PATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH \
      GAZEBO_RESOURCE_PATH PKG_CONFIG_PATH OGRE_RESOURCE_PATH; do
    actual="${!variable}"
    [[ "$actual" != *::* && "$actual" != :* && "$actual" != *: ]] || \
      return 1
    if [[ "$actual" =~ (^|:)[^:]*/(src|source|devel|build)(/|:|$) ]]; then
      return 1
    fi
  done
  return 0
}

bunker_validate_plugin_ldd() {
  [[ $# -eq 3 ]] || return 1
  local -a actual=("$1" "$2" "$3")
  local -a expected=(
    "$bunker_plugin_planar" "$bunker_plugin_laser" "$bunker_plugin_ray")
  local index path canonical
  for index in 0 1 2; do
    path="${actual[$index]}"
    [[ "$path" == "${expected[$index]}" && -f "$path" && \
       ! -L "$path" && -r "$path" ]] || return 1
    canonical="$(/usr/bin/realpath -e -- "$path")" || return 1
    [[ "$canonical" == "$path" ]] || return 1
  done
  bunker_require_variable LD_LIBRARY_PATH || return 1
  [[ -n "$LD_LIBRARY_PATH" && "$LD_LIBRARY_PATH" != :* && \
     "$LD_LIBRARY_PATH" != *: && "$LD_LIBRARY_PATH" != *::* ]] || return 1
  local -a directories=()
  IFS=: read -r -a directories <<< "$LD_LIBRARY_PATH"
  local -A seen=()
  local -a unique=()
  local directory
  for directory in "${directories[@]}"; do
    [[ "$directory" == /* && -d "$directory" && ! -L "$directory" && \
       -r "$directory" && -x "$directory" ]] || return 1
    canonical="$(/usr/bin/realpath -e -- "$directory")" || return 1
    [[ "$canonical" == "$directory" ]] || return 1
    if [[ "$canonical" =~ (^|/)(src|source|devel|build)(/|$) ]]; then
      return 1
    fi
    if [[ -z "${seen[$canonical]+present}" ]]; then
      seen["$canonical"]=1
      unique+=("$canonical")
    fi
  done
  local basename candidate resolved
  local -a matches=()
  for index in 0 1 2; do
    basename="${expected[$index]##*/}"
    matches=()
    for directory in "${unique[@]}"; do
      candidate="$directory/$basename"
      if [[ -e "$candidate" || -L "$candidate" ]]; then
        [[ -f "$candidate" && ! -L "$candidate" ]] || return 1
        resolved="$(/usr/bin/realpath -e -- "$candidate")" || return 1
        [[ "$resolved" == "$candidate" ]] || return 1
        matches+=("$resolved")
      fi
    done
    [[ ${#matches[@]} -eq 1 && \
       "${matches[0]}" == "${expected[$index]}" ]] || return 1
    local ldd_output
    ldd_output="$(/usr/bin/ldd "${expected[$index]}" 2>&1)" || return 1
    [[ "$ldd_output" != *"not found"* ]] || return 1
    printf '%s\tPASS\n' "${expected[$index]}"
  done
}

case "${1:-}" in
  --test-environment)
    [[ $# -eq 1 ]] || bunker_install_fail 64 \
      "--test-environment takes no values"
    bunker_validate_environment || bunker_install_fail 65 \
      "environment contract differs"
    exit 0
    ;;
  --test-plugin-ldd)
    [[ $# -eq 4 ]] || bunker_install_fail 64 \
      "--test-plugin-ldd requires three libraries"
    shift
    bunker_validate_plugin_ldd "$@" || bunker_install_fail 65 \
      "plugin ldd contract differs"
    exit 0
    ;;
  "") ;;
  *) bunker_install_fail 64 "unknown argument" ;;
esac

[[ $# -eq 0 ]] || bunker_install_fail 64 "normal mode takes no arguments"
bunker_validate_environment || bunker_install_fail 65 \
  "environment contract differs"

bunker_vendor_package="$bunker_expected_install/share/bunker_description"
bunker_runtime_package="$bunker_expected_install/share/bunker_sim_runtime"
bunker_observed_vendor="$(rospack find bunker_description)" || \
  bunker_install_fail 66 "cannot resolve bunker_description"
bunker_observed_runtime="$(rospack find bunker_sim_runtime)" || \
  bunker_install_fail 66 "cannot resolve bunker_sim_runtime"
[[ "$bunker_observed_vendor" == "$bunker_vendor_package" && \
   "$bunker_observed_runtime" == "$bunker_runtime_package" ]] || \
  bunker_install_fail 66 "installed package paths differ"
[[ "$bunker_observed_vendor" == \
     "$(/usr/bin/realpath -e -- "$bunker_observed_vendor")" && \
   "$bunker_observed_runtime" == \
     "$(/usr/bin/realpath -e -- "$bunker_observed_runtime")" ]] || \
  bunker_install_fail 66 "package paths are noncanonical"

/usr/bin/python3 -B "$bunker_expected_repo/tools/bunker_assets.py" \
  --config "$bunker_expected_repo/config/bunker_assets.json" \
  --source-root "$bunker_expected_repo/src/vendor/bunker_description" \
  --install-root "$bunker_vendor_package" >/dev/null || \
  bunker_install_fail 66 "source/install asset closure differs"

bunker_renderer="$bunker_runtime_package/scripts/render_bunker_runtime.py"
bunker_preflight="$bunker_runtime_package/scripts/spawn_bunker_preflight.py"
bunker_guard="$bunker_expected_install/lib/bunker_sim_runtime/velocity_guard.py"
for bunker_program in "$bunker_renderer" "$bunker_preflight" "$bunker_guard"; do
  [[ -f "$bunker_program" && ! -L "$bunker_program" && \
     -x "$bunker_program" && \
     "$bunker_program" == "$(/usr/bin/realpath -e -- "$bunker_program")" ]] || \
    bunker_install_fail 66 "installed program differs: $bunker_program"
done

bunker_input="$bunker_vendor_package/urdf/bunker.urdf.xacro"
bunker_output="$BUNKER_RUN_DIR/install-bunker-runtime.urdf"
bunker_mesh_json="$BUNKER_RUN_DIR/.install-mesh-records.json"
[[ ! -e "$bunker_output" && ! -L "$bunker_output" && \
   ! -e "$bunker_mesh_json" && ! -L "$bunker_mesh_json" ]] || \
  bunker_install_fail 66 "install probe output already exists"
umask 077
"$bunker_renderer" "$bunker_input" >"$bunker_output" || \
  bunker_install_fail 66 "installed renderer failed"
chmod 600 "$bunker_output"
[[ -f "$bunker_output" && ! -L "$bunker_output" ]] || \
  bunker_install_fail 66 "rendered output is not regular"
/usr/bin/check_urdf "$bunker_output" >/dev/null 2>&1 || \
  bunker_install_fail 66 "installed render fails check_urdf"
bunker_output_sha="$(/usr/bin/sha256sum "$bunker_output")"
bunker_output_sha="${bunker_output_sha%% *}"
[[ "$bunker_output_sha" == \
   "2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832" ]] || \
  bunker_install_fail 66 "rendered output digest differs"

/usr/bin/python3 -I -B - \
  "$bunker_output" "$bunker_vendor_package" \
  "$bunker_expected_repo/config/bunker_assets.json" "$bunker_mesh_json" <<'PY'
import hashlib
import json
import os
import stat
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

urdf_path, package_text, manifest_text, output_text = map(Path, sys.argv[1:])
package = package_text.resolve(strict=True)
manifest = json.loads(manifest_text.read_text(encoding="utf-8"))
expected = {item["path"]: item["sha256"] for item in manifest["files"]}
root = ET.parse(str(urdf_path)).getroot()
records = []
uris = []
for mesh in root.findall(".//mesh"):
    uri = mesh.get("filename")
    prefix = "package://bunker_description/"
    if not isinstance(uri, str) or not uri.startswith(prefix):
        raise SystemExit("unexpected installed mesh URI")
    relative = uri[len(prefix):]
    path = package / relative
    if path.is_symlink() or not path.is_file():
        raise SystemExit("installed mesh is not regular")
    resolved = path.resolve(strict=True)
    resolved.relative_to(package)
    if resolved != path:
        raise SystemExit("installed mesh is noncanonical")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if expected.get(relative) != digest:
        raise SystemExit("installed mesh hash differs")
    uris.append(uri)
    records.append({"uri": uri, "path": str(path), "sha256": digest})
if len(records) != 17 or len(set(uris)) != 17:
    raise SystemExit("installed runtime mesh closure is not exactly 17")
with output_text.open("x", encoding="utf-8") as stream:
    json.dump(sorted(records, key=lambda item: item["uri"]), stream,
              sort_keys=True, separators=(",", ":"))
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
os.chmod(str(output_text), 0o600)
PY

bunker_validate_plugin_ldd \
  "$bunker_plugin_planar" "$bunker_plugin_laser" "$bunker_plugin_ray" \
  >/dev/null || bunker_install_fail 66 "plugin ldd contract differs"

/usr/bin/python3 -I -B - \
  "$bunker_mesh_json" "$bunker_observed_vendor" "$bunker_observed_runtime" \
  "$bunker_renderer" "$bunker_input" "$bunker_output" \
  "$bunker_plugin_planar" "$bunker_plugin_laser" "$bunker_plugin_ray" <<'PY'
import hashlib
import json
import os
import sys
from pathlib import Path

(mesh_json, vendor, runtime, renderer, source, output,
 planar, laser, ray) = sys.argv[1:]

def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()

environment_names = (
    "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
    "LD_LIBRARY_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
    "GAZEBO_RESOURCE_PATH", "PKG_CONFIG_PATH", "OGRE_RESOURCE_PATH",
    "ROS_ETC_DIR", "ROS_ROOT", "ROSLISP_PACKAGE_DIRECTORIES",
    "GAZEBO_MODEL_DATABASE_URI", "PATH", "LANG", "LC_ALL",
    "TMPDIR", "ROS_HOME", "ROS_LOG_DIR", "GAZEBO_LOG_PATH",
    "IGN_FUEL_CACHE_PATH", "XDG_CACHE_HOME", "XDG_CONFIG_HOME",
    "XDG_DATA_HOME",
)
payload = {
    "schema_version": 1,
    "status": "PASS",
    "environment": {name: os.environ[name] for name in environment_names},
    "packages": {
        "bunker_description": vendor,
        "bunker_sim_runtime": runtime,
    },
    "renderer": {
        "path": renderer,
        "sha256": digest(renderer),
        "input": source,
        "input_sha256": digest(source),
        "output": output,
        "output_sha256": digest(output),
    },
    "meshes": json.loads(Path(mesh_json).read_text(encoding="utf-8")),
    "plugins": [
        {"path": path, "ldd_status": "PASS"}
        for path in (planar, laser, ray)
    ],
}
print(json.dumps(payload, sort_keys=True, separators=(",", ":")))
PY
