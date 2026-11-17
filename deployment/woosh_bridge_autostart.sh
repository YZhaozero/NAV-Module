#!/bin/bash
# Keep the original topic contract; camera1 is throttled before the 100 Mb/s link.
source "$HOME/.bashrc"
source /opt/ros/noetic/setup.bash
source "$HOME/ros2_humble_ws/install/setup.bash"
while ! rosparam list > /dev/null; do sleep 2; done
rosparam load "$HOME/ros2_humble_ws/bridge.yaml"
python3 - <<'PARAMS'
import rosparam
name = 'livox/imu_192_168_128_3'
topics = rosparam.get_param('/topics')
imu = [row for row in topics if row['topic'] == name]
assert len(imu) == 1
camera_name = 'camera_1/color/image_raw'
camera = [row for row in topics if row['topic'] == camera_name]
assert len(camera) == 1
rosparam.set_param_raw('/topics', [row for row in topics if row['topic'] not in (name, camera_name)])
transport = dict(camera[0], topic='nav/camera_1/image/compressed', type='sensor_msgs/msg/CompressedImage')
rosparam.set_param_raw('/nav_camera_bridge/topics', [transport])
rosparam.set_param_raw('/nav/camera_1/image/compressed/format', 'png')
rosparam.set_param_raw('/nav/camera_1/image/compressed/png_level', 1)
rosparam.set_param_raw('/nav_camera_bridge/services_1_to_2', [])
rosparam.set_param_raw('/nav_camera_bridge/services_2_to_1', [])
rosparam.set_param_raw('/nav_imu_bridge/topics', imu)
rosparam.set_param_raw('/nav_imu_bridge/services_1_to_2', [])
rosparam.set_param_raw('/nav_imu_bridge/services_2_to_1', [])
PARAMS
if [ "$?" != 0 ]; then exit 1; fi
nav_bridge_bin="$HOME/ros2_humble_ws/install/ros1_bridge/lib/ros1_bridge/parameter_bridge"
"$nav_bridge_bin" --ros1-start __ns:=/ --ros1-end --ros2-start --ros-args -r __ns:=/foo --ros2-end &
nav_main_pid=$!
sleep 2
"$nav_bridge_bin" --ros1-start __ns:=/ __name:=ros_bridge_imu topics:=/nav_imu_bridge/topics services_1_to_2:=/nav_imu_bridge/services_1_to_2 services_2_to_1:=/nav_imu_bridge/services_2_to_1 --ros1-end --ros2-start --ros-args -r __ns:=/foo -r __node:=ros_bridge_imu --ros2-end &
nav_imu_pid=$!
# Limit frames before lossless PNG transport; Jetson restores the original raw topic.
/opt/ros/noetic/lib/topic_tools/throttle messages /camera_1/color/image_raw 5.0 /nav/camera_1/image_throttled __name:=nav_camera1_throttle &
nav_camera_throttle_pid=$!
/bin/bash -c 'source /opt/ros/noetic/setup.bash; exec /opt/ros/noetic/lib/image_transport/republish raw in:=/nav/camera_1/image_throttled compressed out:=/nav/camera_1/image __name:=nav_camera1_png' &
nav_camera_encoder_pid=$!
"$nav_bridge_bin" --ros1-start __ns:=/ __name:=ros_bridge_camera1 topics:=/nav_camera_bridge/topics services_1_to_2:=/nav_camera_bridge/services_1_to_2 services_2_to_1:=/nav_camera_bridge/services_2_to_1 --ros1-end --ros2-start --ros-args -r __ns:=/foo -r __node:=ros_bridge_camera1 --ros2-end &
nav_camera_bridge_pid=$!
trap 'kill -INT "$nav_camera_bridge_pid" "$nav_camera_encoder_pid" "$nav_camera_throttle_pid" "$nav_imu_pid" "$nav_main_pid"; wait' EXIT
wait -n "$nav_camera_bridge_pid" "$nav_camera_encoder_pid" "$nav_camera_throttle_pid" "$nav_imu_pid" "$nav_main_pid"
exit 1
