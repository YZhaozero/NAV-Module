#!/usr/bin/env bash
set -e
nav_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
nav_runtime_config="${NAV_RUNTIME_CONFIG:-$HOME/.config/nav/runtime.env}"
if [[ -f "$nav_runtime_config" ]]; then
  set -a
  source "$nav_runtime_config"
  set +a
fi
source "$nav_root/scripts/map_environment.sh"
if [[ ! -f "$nav_root/install/local_setup.bash" ]]; then
  echo 'Build this workspace before launching navigation.' >&2
  return 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
nav_deps="${NAV_DEPS_ROOT:-${HOME}/nav_dependencies}/setup.bash"
if [[ -f "$nav_deps" ]]; then source "$nav_deps"; fi
source "$nav_root/scripts/woosh_agent_environment.sh"
nav_meshnav_root="${NAV_MESHNAV_ROOT:-$nav_root/meshnav}"
if [[ -f "$nav_meshnav_root/install/local_setup.bash" ]]; then
  nav_meshnav_lib="$nav_meshnav_root/sysroot/usr/lib/$(uname -m)-linux-gnu"
  if [[ -d "$nav_meshnav_lib" ]]; then export LD_LIBRARY_PATH="$nav_meshnav_lib:${LD_LIBRARY_PATH:-}"; fi
  source "$nav_meshnav_root/install/local_setup.bash"
fi
export NAV_MESHNAV_MAP_DIR="${NAV_MESHNAV_MAP_DIR:-$nav_root/meshnav-maps}"
source "$nav_root/install/local_setup.bash"
set -u
export NAV_INSTALL_PREFIX="$nav_root/install"
