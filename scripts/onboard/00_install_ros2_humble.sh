#!/usr/bin/env bash
# Step 0: ROS 2 Humble (Desktop) on Ubuntu 22.04 (MOREFINE M11 / Intel N305).
# Run ON THE ROBOT PC, not on the development machine.
# Set USE_CN_MIRROR=1 to use the TUNA mirror (recommended in mainland China).
set -euo pipefail
source "$(dirname "$0")/common.sh"
require_jammy

sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
if [[ "${USE_CN_MIRROR:-0}" == "1" ]]; then
  ROS_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
else
  ROS_MIRROR="http://packages.ros.org/ros2/ubuntu"
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions \
  python3-rosdep python3-vcstool

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update || warn "rosdep update failed (network?). Retry later or use rosdepc in China."

append_once 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc"
append_once 'export ROS_DOMAIN_ID=42' "$HOME/.bashrc"
info "ROS 2 Humble installed. Open a new shell or: source ~/.bashrc"
