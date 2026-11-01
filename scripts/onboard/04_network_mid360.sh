#!/usr/bin/env bash
# Step 4: static IP on the Ethernet port wired to the Livox Mid-360.
# Mid-360 factory IP is 192.168.1.1XX (XX = last two digits of its serial number);
# the host must be in 192.168.1.0/24. We use 192.168.1.50 (must match config/MID360_config.json).
#   LIDAR_IFACE=enp2s0 ./04_network_mid360.sh
set -euo pipefail
source "$(dirname "$0")/common.sh"

IFACE="${LIDAR_IFACE:-}"
HOST_IP="${LIDAR_HOST_IP:-192.168.1.50}"
if [[ -z "$IFACE" ]]; then
  echo "Ethernet interfaces:"; ip -br link | awk '$1 ~ /^(en|eth)/'
  die "Set LIDAR_IFACE=<iface> (the port connected to the Mid-360)."
fi

if command -v nmcli >/dev/null; then
  sudo nmcli con delete livox-mid360 >/dev/null 2>&1 || true
  sudo nmcli con add type ethernet ifname "$IFACE" con-name livox-mid360 \
    ipv4.method manual ipv4.addresses "${HOST_IP}/24" ipv4.never-default yes ipv6.method ignore
  sudo nmcli con up livox-mid360
else
  die "NetworkManager (nmcli) not found; configure ${HOST_IP}/24 on ${IFACE} via netplan."
fi

# Bigger socket buffers help with 200k pts/s UDP streams.
echo -e "net.core.rmem_max=26214400\nnet.core.rmem_default=26214400" | sudo tee /etc/sysctl.d/90-livox.conf >/dev/null
sudo sysctl --system >/dev/null
info "Host ${HOST_IP}/24 on ${IFACE}. Test: ping 192.168.1.1XX (your LiDAR)."
