#!/bin/bash

set -euo pipefail

if [[ $# -ne 0 ]]; then
  echo "P450 MID360 smoke does not accept arguments" >&2
  exit 64
fi
if [[ "${P450_SENSOR_PROFILE+x}" == "x" ]] && \
   [[ "$P450_SENSOR_PROFILE" != "mid360" ]]; then
  echo "P450_SENSOR_PROFILE conflicts with the MID360 entrypoint" >&2
  exit 64
fi

P450_SENSOR_PROFILE=mid360
export P450_SENSOR_PROFILE
p450_entrypoint_path="$(/usr/bin/realpath -e -- "${BASH_SOURCE[0]}")"
p450_entrypoint_dir="${p450_entrypoint_path%/*}"
exec "$p450_entrypoint_dir/smoke_p450_standalone.bash"
