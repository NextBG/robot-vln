#!/usr/bin/env bash
# Step 1: apt + pip dependencies for chassis / camera / GNSS / LiDAR / VLN client.
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

sudo apt-get update
sudo apt-get install -y \
  git build-essential cmake python3-pip v4l-utils can-utils net-tools ethtool htop \
  libopus-dev ffmpeg \
  ros-humble-usb-cam ros-humble-image-transport-plugins ros-humble-camera-calibration \
  ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-view \
  ros-humble-nmea-msgs ros-humble-nmea-navsat-driver ros-humble-nav2-msgs ros-humble-tf2-geometry-msgs \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-xacro ros-humble-launch-xml ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joy ros-humble-joy-teleop ros-humble-joy-linux \
  ros-humble-teleop-twist-keyboard ros-humble-rmw-cyclonedds-cpp \
  python3-numpy python3-opencv python3-requests python3-serial python3-transforms3d python3-pytest

# RoboMaster SDK (jeguzzi fork, required by robomaster_ros). Ubuntu 22.04 ships Python 3.10.
python3 -m pip install --user -U numpy-quaternion pyyaml
python3 -m pip install --user "git+https://github.com/jeguzzi/RoboMaster-SDK.git"
python3 -m pip install --user "git+https://github.com/jeguzzi/RoboMaster-SDK.git#egg=rm_libmedia_codec&subdirectory=lib/libmedia_codec"

# serial port access for the GNSS module
sudo usermod -aG dialout,video "$USER"

append_once 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$HOME/.bashrc"
info "Dependencies installed. Log out/in once so the dialout/video group change applies."
