#!/usr/bin/env bash
# Step 1: apt + pip dependencies for chassis / camera / GNSS / LiDAR / VLN client.
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

sudo apt-get update
sudo apt-get install -y \
  git build-essential cmake python3-pip python3-vcstool python3-rosdep python3-colcon-common-extensions v4l-utils can-utils net-tools ethtool htop \
  libopus-dev ffmpeg \
  ros-humble-usb-cam ros-humble-image-transport-plugins ros-humble-camera-calibration \
  ros-humble-cv-bridge ros-humble-vision-opencv ros-humble-image-view \
  ros-humble-nmea-msgs ros-humble-nmea-navsat-driver ros-humble-nav2-msgs ros-humble-tf2-geometry-msgs \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-xacro ros-humble-launch-xml ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joy ros-humble-joy-teleop ros-humble-joy-linux \
  ros-humble-teleop-twist-keyboard ros-humble-rmw-cyclonedds-cpp \
  python3-numpy python3-opencv python3-requests python3-serial python3-transforms3d python3-pytest

# ROS Python deps go to <repo>/.pydeps (used via scripts/onboard/ros_env.sh), NOT ~/.local:
# ROS 2 Humble's compiled modules need NumPy 1.x, while other projects on the PC may use NumPy 2.
PYDEPS="${REPO_ROOT}/.pydeps"
USER_SITE="$(python3 -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$PYDEPS"
python3 -m pip install --target "$PYDEPS" -U "numpy>=1.25,<2" numpy-quaternion "transforms3d>=0.4"  # apt transforms3d 0.3 breaks on numpy>=1.24
# RoboMaster SDK (jeguzzi fork, required by robomaster_ros). Ubuntu 22.04 ships Python 3.10.
if ! PYTHONNOUSERSITE=1 PYTHONPATH="$PYDEPS" python3 -c "import robomaster" 2>/dev/null; then
  python3 -m pip install --target "$PYDEPS" --no-deps \
    "git+https://github.com/jeguzzi/RoboMaster-SDK.git" netaddr myqr \
    || { warn "GitHub install failed; copying robomaster from $USER_SITE"; cp -r "$USER_SITE"/robomaster "$USER_SITE"/netaddr "$PYDEPS"/; }
fi
# Video decoder (only needed for the S1 camera stream). Ubuntu's old pip/packaging can't build it
# with the current scikit-build-core, so reuse an existing build and never fail the whole setup.
if ! PYTHONNOUSERSITE=1 PYTHONPATH="$PYDEPS" python3 -c "import libmedia_codec" 2>/dev/null; then
  if ls "$USER_SITE"/libmedia_codec*.so >/dev/null 2>&1; then
    cp "$USER_SITE"/libmedia_codec*.so "$PYDEPS"/
  else
    python3 -m pip install --user -U "pip>=24"
    python3 -m pip install --target "$PYDEPS" "git+https://github.com/jeguzzi/RoboMaster-SDK.git#egg=rm_libmedia_codec&subdirectory=lib/libmedia_codec" \
      || warn "rm_libmedia_codec failed to build; S1 camera video disabled (run robomaster_ros with video:=-1)"
  fi
fi

# serial port access for the GNSS module
sudo usermod -aG dialout,video "$USER"

append_once 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' "$HOME/.bashrc"
info "Dependencies installed. Log out/in once so the dialout/video group change applies."
