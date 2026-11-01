#!/usr/bin/env bash
# Step 5: fetch third-party sources and build the ROS 2 workspace.
set -euo pipefail
source "$(dirname "$0")/common.sh"
source_ros

cd "$WS"
mkdir -p src
vcs import src < robot.repos
if [[ "${WITH_CAN:-0}" == "1" ]]; then
  vcs import --recursive src < can.repos
fi

# livox_ros_driver2 ships package_ROS1.xml / package_ROS2.xml; its build.sh normally copies one.
LIVOX="src/third_party/livox_ros_driver2"
if [[ -f "$LIVOX/package_ROS2.xml" ]]; then
  cp -f "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"
fi

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || warn "some rosdep keys unresolved"

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble

append_once "source ${WS}/install/setup.bash" "$HOME/.bashrc"
info "Workspace built. source ${WS}/install/setup.bash"
