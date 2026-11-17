#!/usr/bin/env bash
# Optional user-owned overlay installed by deployment/install_woosh_agent.py.
nav_woosh_root="${NAV_WOOSH_AGENT_ROOT:-${HOME}/nav_dependencies/woosh-agent-0.0.7}"
if [[ -f "$nav_woosh_root/opt/ros/humble/share/ament_index/resource_index/packages/woosh_robot_agent" ]]; then
  export AMENT_PREFIX_PATH="$nav_woosh_root/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export PYTHONPATH="$nav_woosh_root/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="$nav_woosh_root/opt/ros/humble/lib:$nav_woosh_root/opt/woosh/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
unset nav_woosh_root
