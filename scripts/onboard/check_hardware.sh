#!/usr/bin/env bash
# Quick hardware sanity check on the robot PC (no ROS needed).
source "$(dirname "$0")/common.sh"
ok=0
check() { if eval "$2" >/dev/null 2>&1; then info "OK   $1"; else warn "FAIL $1"; ok=1; fi; }

check "ROS 2 Humble installed"            "test -f /opt/ros/humble/setup.bash"
check "Livox-SDK2 installed"              "ls /usr/local/lib/liblivox_lidar_sdk_shared.so"
check "GNSS /dev/wheeltec_gps"            "test -e /dev/wheeltec_gps"
check "Webcam /dev/vln_cam"               "test -e /dev/vln_cam"
check "S1 RNDIS link (192.168.42.2)"      "ping -c1 -W1 192.168.42.2"
check "Mid-360 reachable (192.168.1.1XX)" "ip neigh | grep -q '192.168.1.1[0-9][0-9]' || ping -c1 -W1 -b 192.168.1.255"
check "robomaster SDK importable"         "python3 -c 'import robomaster'"

if [[ -e /dev/vln_cam ]]; then v4l2-ctl -d /dev/vln_cam --list-formats-ext | head -n 30; fi
exit $ok
