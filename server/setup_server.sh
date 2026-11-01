#!/usr/bin/env bash
# GPU inference server setup for InternVLA-N1 (DualVLN).
# Run ON THE GPU SERVER (Linux + NVIDIA driver for CUDA 12.x, >=24 GB VRAM).
#   bash server/setup_server.sh
# Env vars:
#   INTERNNAV_ROOT  (default ~/InternNav)
#   CN_MIRROR=1     use Tsinghua PyPI + hf-mirror.com (default 1; set 0 outside mainland China)
#   MODEL=InternVLA-N1-DualVLN   (or InternVLA-N1-w-NavDP / InternVLA-N1-System2)
#   GH_PROXY=https://ghproxy.example/   optional prefix for github.com downloads
set -euo pipefail
ROOT="${INTERNNAV_ROOT:-$HOME/InternNav}"
HERE="$(cd "$(dirname "$0")" && pwd)"
MODEL="${MODEL:-InternVLA-N1-DualVLN}"
CN_MIRROR="${CN_MIRROR:-1}"
GH="${GH_PROXY:-}https://github.com"
log() { echo -e "\033[1;32m[setup]\033[0m $*"; }

nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv || { echo "No NVIDIA GPU"; exit 1; }
if ! command -v git >/dev/null || ! command -v wget >/dev/null; then
  apt-get update && apt-get install -y git wget || sudo apt-get install -y git wget
fi

if [[ "$CN_MIRROR" == "1" ]]; then
  export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
  export HF_ENDPOINT=https://hf-mirror.com
fi

# 1. Python 3.10 env (conda if present, else install Miniconda)
if ! command -v conda >/dev/null; then
  for c in "$HOME/miniconda3" /opt/conda /root/miniconda3; do
    [[ -x "$c/bin/conda" ]] && export PATH="$c/bin:$PATH" && break
  done
fi
if ! command -v conda >/dev/null; then
  log "Installing Miniconda"
  MC_URL=https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
  [[ "$CN_MIRROR" == "1" ]] && MC_URL=https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh
  wget -q "$MC_URL" -O /tmp/mc.sh && bash /tmp/mc.sh -b -p "$HOME/miniconda3"
  export PATH="$HOME/miniconda3/bin:$PATH"
fi
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main >/dev/null 2>&1 || true
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r >/dev/null 2>&1 || true
conda env list | grep -q '^internnav ' || conda create -y -n internnav python=3.10
conda activate internnav
python -m pip install -U pip wheel setuptools

# 2. Source
if [[ ! -d "$ROOT/.git" ]]; then
  log "Cloning InternNav"
  git clone "$GH/InternRobotics/InternNav.git" "$ROOT"
fi
cd "$ROOT"
git submodule update --init --recursive || log "submodule update failed (LongCLIP is not needed for InternVLA-N1)"

# 3. PyTorch 2.6 + CUDA 12.4 (matches flash-attn 2.7.4.post1 prebuilt wheel)
TORCH_PKGS=(torch==2.6.0 torchvision==0.21.0)
if ! python -c "import torch,sys; sys.exit(0 if torch.__version__.startswith('2.6') else 1)" 2>/dev/null; then
  if [[ "$CN_MIRROR" == "1" ]]; then
    pip install "${TORCH_PKGS[@]}" -f https://mirrors.aliyun.com/pytorch-wheels/cu124/
  else
    pip install "${TORCH_PKGS[@]}" --index-url https://download.pytorch.org/whl/cu124
  fi
fi

# 4. flash-attn (required: the agent loads the model with attn_implementation=flash_attention_2)
if ! python -c "import flash_attn" 2>/dev/null; then
  FA_WHL=flash_attn-2.7.4.post1+cu12torch2.6cxx11abiFALSE-cp310-cp310-linux_x86_64.whl
  pip install "$GH/Dao-AILab/flash-attention/releases/download/v2.7.4.post1/$FA_WHL" || \
    MAX_JOBS=8 pip install flash-attn==2.7.4.post1 --no-build-isolation
fi

# 5. InternNav + InternVLA-N1 extras (+ server deps)
sed -i 's#git+https://github.com#git+'"$GH"'#g' requirements/internvla_n1.txt
pip install -e ".[internvla_n1]"
pip install flask "huggingface_hub[cli]" qwen-vl-utils

# 6. Checkpoint
mkdir -p checkpoints
huggingface-cli download "InternRobotics/$MODEL" --local-dir "checkpoints/$MODEL"

# 7. Our HTTP wrapper
cp -f "$HERE/internvla_n1_server.py" "$ROOT/scripts/realworld/internvla_n1_server.py"
python -c "import torch, flash_attn, transformers; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'transformers', transformers.__version__)"
log "Done. Start the server:"
echo "  conda activate internnav && cd $ROOT && python scripts/realworld/internvla_n1_server.py --model_path checkpoints/$MODEL --intrinsic FX FY CX CY --port 5801"
