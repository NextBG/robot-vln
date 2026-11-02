#!/usr/bin/env bash
# CityWalker local planner on the GPU server (reuses the InternNav conda env: torch, flask, cv2, yaml).
#   bash server/setup_citywalker.sh                 # then start, see docs/vln/citywalker.md
# GitHub is slow from the cloud box: if a download stalls, fetch locally and scp to the same paths:
#   gh api repos/ai4ce/CityWalker/tarball/main > citywalker.tgz
#   gh api repos/facebookresearch/dinov2/tarball/main > dinov2.tgz
#   https://github.com/ai4ce/CityWalker/releases/download/v1.0/CityWalker_2000hr.ckpt  (1.75 GB)
set -euo pipefail
ROOT=${ROOT:-/root}
ENV_PY=${ENV_PY:-/root/miniconda3/envs/internnav/bin/python}
GH=${GH_PROXY:-}https://github.com

fetch_repo() {  # name url dir
  if [ -d "$3/.git" ] || [ -f "$3/README.md" ]; then echo "[ok] $1 at $3"; return; fi
  git clone --depth 1 "$2" "$3"
}
fetch_repo CityWalker "$GH/ai4ce/CityWalker.git" "$ROOT/CityWalker"
fetch_repo dinov2 "$GH/facebookresearch/dinov2.git" "$ROOT/dinov2"

CKPT=$ROOT/CityWalker/checkpoints/CityWalker_2000hr.ckpt
mkdir -p "$(dirname "$CKPT")"
if [ ! -s "$CKPT" ] || [ "$(stat -c %s "$CKPT")" -lt 1800000000 ]; then
  wget -c -O "$CKPT" "$GH/ai4ce/CityWalker/releases/download/v1.0/CityWalker_2000hr.ckpt"
fi
"$ENV_PY" -c "import torch, flask, cv2, yaml; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
echo "start: $ENV_PY $(dirname "$0")/citywalker_server.py --citywalker_root $ROOT/CityWalker --dinov2_root $ROOT/dinov2 --ckpt $CKPT"
