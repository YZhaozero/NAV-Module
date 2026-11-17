#!/usr/bin/env bash
set -eo pipefail
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$nav_root/scripts/runtime_environment.sh"
display_exports="$(python3 "$nav_root/scripts/device_service.py" display --wait 45)"
eval "$display_exports"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OMP_WAIT_POLICY="${OMP_WAIT_POLICY:-PASSIVE}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
if [[ "${NAV_SENSOR_PROFILE:-}" == "woosh" ]]; then
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  exec ros2 launch nav_app woosh_app.launch.py fullscreen:=true phone_remote_enabled:=true
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-101}"
exec ros2 launch nav_app ar_nav.launch.py fullscreen:=true phone_remote_enabled:=true
