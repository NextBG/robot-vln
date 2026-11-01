#!/usr/bin/env bash
# Source this (not execute) before any ros2 command on the robot:
#   source ~/robot-vln/scripts/onboard/ros_env.sh
# ROS 2 Humble needs NumPy 1.x. Other projects on this PC keep NumPy 2 in ~/.local, so the ROS
# Python deps (numpy<2, numpy-quaternion, robomaster SDK) live in <repo>/.pydeps and the user
# site-packages are disabled for ROS processes.
_RV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
_rv_u=$-; set +u
source /opt/ros/humble/setup.bash
[[ -f "${_RV_ROOT}/ros2_ws/install/setup.bash" ]] && source "${_RV_ROOT}/ros2_ws/install/setup.bash"
[[ $_rv_u == *u* ]] && set -u
export PYTHONNOUSERSITE=1
export PYTHONPATH="${_RV_ROOT}/.pydeps${PYTHONPATH:+:${PYTHONPATH}}"
unset _RV_ROOT _rv_u
