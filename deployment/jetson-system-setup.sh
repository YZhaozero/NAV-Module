#!/usr/bin/env bash
# Ubuntu 22.04 / ROS Humble administrator setup for the nvidia Jetson account.
set -euo pipefail

if (( EUID != 0 )); then
  echo "Run with sudo: sudo bash $0" >&2
  exit 1
fi

usermod -aG dialout nvidia
# Existing desktop/systemd sessions retain their old supplementary groups.
# The ACL grants the same access now; dialout membership applies after login/reboot.
for nav_serial in /dev/ttyTHS*; do
  if [[ -c "$nav_serial" ]]; then
    setfacl -m u:nvidia:rw "$nav_serial"
  fi
done

apt-get update
apt-get install -y \
  ninja-build libgoogle-glog-dev libfmt-dev libsqlite3-dev libusb-1.0-0-dev \
  python3-serial python3-pytest python3-pyqt5 python3-scipy python3-opencv \
  acl x11-utils x11-xserver-utils \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-gtsam ros-humble-sophus ros-humble-librealsense2 \
  ros-humble-pcl-ros ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-diagnostic-updater ros-humble-ament-cmake-auto \
  ros-humble-rosbag2 ros-humble-rosbag2-py

id nvidia
getfacl /dev/ttyTHS*
