#!/usr/bin/env bash
# Step 5: fetch third-party sources and build the ROS 2 workspace.
set -euo pipefail
source "$(dirname "$0")/common.sh"
source_ros

cd "$WS"
mkdir -p src
# SKIP_VCS=1 when sources were copied in manually (robot without reliable GitHub access).
if [[ "${SKIP_VCS:-0}" != "1" ]]; then
  vcs import src < robot.repos
  if [[ "${WITH_CAN:-0}" == "1" ]]; then
    vcs import --recursive src < can.repos
  fi
fi

# livox_ros_driver2 ships package_ROS1.xml / package_ROS2.xml; its build.sh normally copies one.
LIVOX="src/third_party/livox_ros_driver2"
if [[ -f "$LIVOX/package_ROS2.xml" ]]; then
  cp -f "$LIVOX/package_ROS2.xml" "$LIVOX/package.xml"
fi

[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update || warn "rosdep update failed"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || warn "some rosdep keys unresolved"

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

append_once "source ${REPO_ROOT}/scripts/onboard/ros_env.sh" "$HOME/.bashrc"
info "Workspace built. source ${REPO_ROOT}/scripts/onboard/ros_env.sh"
