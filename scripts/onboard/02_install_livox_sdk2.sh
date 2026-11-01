#!/usr/bin/env bash
# Step 2: Livox-SDK2 (system-wide lib needed by livox_ros_driver2 for Mid-360).
set -euo pipefail
source "$(dirname "$0")/common.sh"

SRC="${HOME}/thirdparty/Livox-SDK2"
if [[ ! -d "$SRC" ]]; then
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$SRC"
fi
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SRC/build" -j"$(nproc)"
sudo cmake --install "$SRC/build"
sudo ldconfig
info "Livox-SDK2 installed to /usr/local (liblivox_lidar_sdk_shared.so)."
