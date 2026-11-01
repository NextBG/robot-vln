#!/usr/bin/env bash
# GPU inference server setup for InternVLA-N1 (run on a Linux machine with an NVIDIA GPU, >=24 GB VRAM recommended).
# Follows the InternNav installation guide; check https://github.com/InternRobotics/InternNav for updates.
set -euo pipefail
ROOT="${INTERNNAV_ROOT:-$HOME/InternNav}"
HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ ! -d "$ROOT" ]]; then
  git clone --recursive https://github.com/InternRobotics/InternNav.git "$ROOT"
fi
cd "$ROOT"

if ! conda env list | grep -q '^internnav '; then
  conda create -y -n internnav python=3.10
fi
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate internnav

pip install torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
pip install -e ".[model]" || pip install -e .
pip install flask pillow opencv-python-headless huggingface_hub

# Use HF_ENDPOINT=https://hf-mirror.com in mainland China.
mkdir -p checkpoints
huggingface-cli download InternRobotics/InternVLA-N1-DualVLN --local-dir checkpoints/InternVLA-N1-DualVLN

cp -f "$HERE/internvla_n1_server.py" "$ROOT/scripts/realworld/internvla_n1_server.py"
echo "Done. Start with:"
echo "  conda activate internnav && cd $ROOT && python scripts/realworld/internvla_n1_server.py --intrinsic FX FY CX CY"
