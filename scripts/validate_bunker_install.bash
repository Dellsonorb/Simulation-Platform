#!/bin/bash

set -euo pipefail

usage() {
  cat <<'EOF'
usage: scripts/validate_bunker_install.bash

Check the installed BUNKER packages, runtime URDF, launch file, and Gazebo
plugins. Run this through scripts/with_bunker_env.bash.
EOF
}

if [[ "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
[[ $# -eq 0 ]] || { usage >&2; exit 64; }

script_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
repo_root="${script_path%/scripts/*}"
install_root="$repo_root/install/p450-clean"
run_dir="${BUNKER_RUN_DIR:-$repo_root/logs/bunker_install_check}"

description="$(rospack find bunker_description)"
runtime="$(rospack find bunker_sim_runtime)"
[[ "$description" == "$install_root/share/bunker_description" ]]
[[ "$runtime" == "$install_root/share/bunker_sim_runtime" ]]

renderer="$runtime/scripts/render_bunker_runtime.py"
guard="$install_root/lib/bunker_sim_runtime/velocity_guard.py"
for program in "$renderer" "$guard"; do
  [[ -x "$program" ]] || {
    echo "validate-bunker-install: missing executable $program" >&2
    exit 66
  }
done

/usr/bin/mkdir -p -- "$run_dir"
rendered="$(/usr/bin/mktemp -p "$run_dir" bunker-runtime.XXXXXX.urdf)"
cleanup() { /usr/bin/rm -f -- "$rendered"; }
trap cleanup EXIT
"$renderer" "$description/urdf/bunker.urdf.xacro" >"$rendered"
check_urdf "$rendered" >/dev/null
roslaunch --files bunker_sim_runtime bunker_standalone.launch >/dev/null

plugins=(
  "$install_root/lib/libbunker_planar_move_plugin.so"
  /opt/ros/noetic/lib/libgazebo_ros_laser.so
  /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so
)
for plugin in "${plugins[@]}"; do
  [[ -r "$plugin" ]] || {
    echo "validate-bunker-install: missing Gazebo plugin $plugin" >&2
    exit 66
  }
  if /usr/bin/ldd "$plugin" | /bin/grep -q 'not found'; then
    echo "validate-bunker-install: unresolved dependency in $plugin" >&2
    exit 66
  fi
done

echo "PASS: installed BUNKER model, launch, and Gazebo plugins"
