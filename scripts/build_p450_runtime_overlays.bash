#!/bin/bash

set -euo pipefail

p450_overlay_validate_output_path() {
  local p450_output_repository="$1"
  local p450_output_relative="$2"
  local p450_output_current
  local p450_output_part
  local p450_output_resolved
  local -a p450_output_parts

  if ! p450_output_repository="$({
    /usr/bin/realpath -e -- "$p450_output_repository"
  } 2>/dev/null)" || [[ ! -d "$p450_output_repository" ]]; then
    echo "overlay repository must name an existing directory" >&2
    return 66
  fi
  if [[ -z "$p450_output_relative" || "$p450_output_relative" == /* ||
      "$p450_output_relative" == *:* ||
      "$p450_output_relative" == *$'\n'* ||
      "$p450_output_relative" == *$'\r'* ]]; then
    echo "overlay output must be a safe relative path" >&2
    return 64
  fi

  IFS='/' read -r -a p450_output_parts <<<"$p450_output_relative"
  p450_output_current="$p450_output_repository"
  for p450_output_part in "${p450_output_parts[@]}"; do
    if [[ -z "$p450_output_part" || "$p450_output_part" == "." ||
        "$p450_output_part" == ".." ]]; then
      echo "overlay output must be a normalized relative path" >&2
      return 64
    fi
    p450_output_current="$p450_output_current/$p450_output_part"
    if [[ -L "$p450_output_current" ]]; then
      echo "overlay output path contains a symbolic link: $p450_output_current" >&2
      return 65
    fi
    if [[ -e "$p450_output_current" && ! -d "$p450_output_current" ]]; then
      echo "overlay output path component is not a directory: $p450_output_current" >&2
      return 65
    fi
  done

  p450_output_resolved="$(
    /usr/bin/realpath -m -- "$p450_output_repository/$p450_output_relative"
  )"
  case "$p450_output_resolved" in
    "$p450_output_repository"/*) ;;
    *)
      echo "overlay output path escapes the SIM repository" >&2
      return 65
      ;;
  esac
  printf '%s\n' "$p450_output_resolved"
}

if [[ "${1:-}" == "--test-output-path" ]]; then
  [[ $# -eq 3 ]] || exit 64
  p450_overlay_validate_output_path "$2" "$3"
  exit $?
fi

if [[ -z "${P450_PX4_ROOT:-}" ]]; then
  echo "P450_PX4_ROOT must name the pinned external PX4 checkout" >&2
  exit 64
fi

p450_overlay_wrapper="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
p450_overlay_script_dir="${p450_overlay_wrapper%/*}"
p450_overlay_repo_root="${p450_overlay_script_dir%/*}"
p450_overlay_config="$p450_overlay_repo_root/config/p450_runtime.json"
p450_overlay_validator="$p450_overlay_repo_root/tools/p450_runtime.py"
p450_overlay_noetic="$p450_overlay_repo_root/scripts/with_noetic_env.bash"
p450_overlay_project="$p450_overlay_repo_root/runtime_overlays/px4_gazebo_plugins"
p450_overlay_build="$(p450_overlay_validate_output_path \
  "$p450_overlay_repo_root" "build/p450-runtime-overlays")"
p450_overlay_install="$(p450_overlay_validate_output_path \
  "$p450_overlay_repo_root" "install/p450-runtime-overlays")"

for p450_overlay_required in \
  "$p450_overlay_config" \
  "$p450_overlay_validator" \
  "$p450_overlay_noetic" \
  "$p450_overlay_project/CMakeLists.txt"; do
  if [[ ! -f "$p450_overlay_required" ]]; then
    echo "P450 overlay build prerequisite is missing: $p450_overlay_required" >&2
    exit 66
  fi
done

p450_overlay_px4_root="$(
  /usr/bin/env -i \
    PATH=/usr/bin:/bin:/usr/sbin:/sbin \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    /usr/bin/python3 -I -B "$p450_overlay_validator" \
      --config "$p450_overlay_config" \
      --px4-root "$P450_PX4_ROOT" \
      --repository-root "$p450_overlay_repo_root" \
      --external-only
)"

"$p450_overlay_noetic" /usr/bin/cmake \
  -S "$p450_overlay_project" \
  -B "$p450_overlay_build" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$p450_overlay_install" \
  -DP450_PX4_ROOT="$p450_overlay_px4_root"
"$p450_overlay_noetic" /usr/bin/cmake --build "$p450_overlay_build"
"$p450_overlay_noetic" /usr/bin/cmake --install "$p450_overlay_build"

/usr/bin/env -i \
  PATH=/usr/bin:/bin:/usr/sbin:/sbin \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  /usr/bin/python3 -I -B "$p450_overlay_validator" \
    --config "$p450_overlay_config" \
    --px4-root "$p450_overlay_px4_root" \
    --repository-root "$p450_overlay_repo_root" >/dev/null

echo "P450 runtime overlays are ready: $p450_overlay_install/lib"
