#!/usr/bin/env bash
# Step 3: udev rules -> /dev/wheeltec_gps (G60) and /dev/vln_cam (webcam).
set -euo pipefail
source "$(dirname "$0")/common.sh"

sudo install -m 644 "$(dirname "$0")/99-vln-robot.rules" /etc/udev/rules.d/99-vln-robot.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 1
ls -l /dev/wheeltec_gps /dev/vln_cam 2>/dev/null || warn "Symlinks not present yet: plug the devices in, check 'lsusb' and edit the rules."
