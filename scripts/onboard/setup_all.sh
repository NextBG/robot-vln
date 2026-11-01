#!/usr/bin/env bash
# Run all onboard setup steps in order (on the robot PC: MOREFINE M11, Ubuntu 22.04).
#   git clone https://github.com/NextBG/robot-vln.git ~/robot-vln
#   cd ~/robot-vln && LIDAR_IFACE=enp1s0 USE_CN_MIRROR=1 ./scripts/onboard/setup_all.sh
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
"$D/00_install_ros2_humble.sh"
"$D/01_install_deps.sh"
"$D/02_install_livox_sdk2.sh"
"$D/03_udev.sh"
if [[ -n "${LIDAR_IFACE:-}" ]]; then "$D/04_network_mid360.sh"; else echo "[skip] 04_network_mid360.sh (set LIDAR_IFACE)"; fi
"$D/05_build_ws.sh"
"$D/check_hardware.sh" || true
