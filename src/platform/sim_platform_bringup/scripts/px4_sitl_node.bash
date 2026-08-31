#!/bin/bash

set -euo pipefail

if [[ -z "${P450_PX4_ROOT:-}" ]]; then
  echo "P450_PX4_ROOT must be set by scripts/with_p450_env.bash" >&2
  exit 64
fi

if [[ "$P450_PX4_ROOT" != /* ]]; then
  echo "P450_PX4_ROOT must be an absolute canonical directory" >&2
  exit 65
fi

if ! p450_px4_canonical="$(/usr/bin/realpath -e -- "$P450_PX4_ROOT")" || \
   [[ ! -d "$p450_px4_canonical" ]] || \
   [[ "$p450_px4_canonical" != "$P450_PX4_ROOT" ]]; then
  echo "P450_PX4_ROOT must be an absolute canonical directory" >&2
  exit 65
fi

p450_px4_binary="$p450_px4_canonical/build/amovlab_sitl_default/bin/px4"
if [[ ! -f "$p450_px4_binary" || ! -x "$p450_px4_binary" ]]; then
  echo "PX4 SITL binary is missing or not executable: $p450_px4_binary" >&2
  exit 66
fi

exec "$p450_px4_binary" "$@"
