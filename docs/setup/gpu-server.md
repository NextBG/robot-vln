# GPU 推理服务器（InternVLA-N1）

## 要求

- Linux + NVIDIA GPU，建议 **≥24 GB 显存**（估计值，7B Qwen2.5-VL + System1），CUDA 12.x 驱动
- conda
- 与机器人在同一局域网，开放 TCP 5801

## 安装

```bash
git clone https://github.com/NextBG/robot-vln.git && cd robot-vln
bash server/setup_server.sh          # 默认 CN_MIRROR=1：清华 PyPI + 阿里 PyTorch + hf-mirror
# GitHub 访问不了时加代理前缀：GH_PROXY=https://<代理>/ bash server/setup_server.sh
# 选择其他权重：MODEL=InternVLA-N1-w-NavDP bash server/setup_server.sh
```

脚本会：clone InternNav → 创建 conda 环境 `internnav` → 安装依赖 → 下载 `InternRobotics/InternVLA-N1-DualVLN` 权重 → 复制本仓库的服务端封装。
**依赖版本以 InternNav 官方安装文档为准**，如果脚本中的 torch 版本和官方不一致，请按官方修改。

## 启动

```bash
conda activate internnav && cd ~/InternNav
python scripts/realworld/internvla_n1_server.py \
  --model_path checkpoints/InternVLA-N1-DualVLN \
  --intrinsic <fx> <fy> <cx> <cy> --port 5801
curl http://localhost:5801/health
python ~/robot-vln/server/test_client.py --instruction "Walk to the door and stop."   # 冒烟测试
```

云 GPU（优云智算 compshare / AutoDL 等）一般只开放 SSH 端口，可以用 SSH 隧道把服务暴露给机器人：

```bash
# 在 M11 上执行，之后 vln_client 的 server_url 用 http://127.0.0.1:5801/eval_dual
ssh -N -L 5801:127.0.0.1:5801 -p <端口> root@<服务器地址>
```

`--intrinsic` 填 `camera_info.yaml` 里标定得到的内参。

### 实测（2026-11-01，compshare RTX 4090）

- 显存占用约 16.6 GB；单步推理约 0.26–0.33 s（640×480 图像，`test_client.py` 测试返回 HTTP 200）。
- 首次启动时修了以下问题，已写进 `setup_server.sh`：
  - 缺少 `numpy-quaternion`
  - diffusers 0.33.1 的 LuminaNextDiT 不兼容，降到 0.32.2
  - LongCLIP 需要 `pkg_resources`，要求 `setuptools<70`
  - 缺少 `checkpoints/depth_anything_v2_metric_hypersim_vits.pth`
- 服务器在后台运行：`nohup python scripts/realworld/internvla_n1_server.py ... > /root/server.log 2>&1 &`
- 不用的时候在控制台关机，避免按时计费。

## 和官方 server 的区别（`server/internvla_n1_server.py`）

- 指令从请求 json 的 `instruction` 字段读取（官方脚本里是写死的）
- 内参由命令行传入
- `depth` 可选（没有就用全 0）
- 增加 `/health` 接口
- **深度补全**（`--depth_fill mono`，默认开启；`--depth_fill none` 关闭）：Mid-360 投影到图像上只覆盖约 3% 的像素，所以用 Depth-Anything-V2 metric（vits，`--mono_depth_ckpt`）算单目深度，按 LiDAR 像素取中值比例对齐尺度，再填补空洞。没有 LiDAR 数据时直接用单目深度（scale=1）。约 28 ms/帧；响应里的 `depth` 字段返回 `lidar_frac`、`mono_scale`、`fill_s`。
- 推理放在 `torch.no_grad()` 中执行：官方 `step_s1`（轨迹扩散）没有关梯度，24 GB 显卡跑到第 4 步左右就会 OOM。修复后连续 20 步显存稳定在约 18.5 GB；S2 单步约 0.2–0.5 s，S2 生成像素目标的那一步约 1.3 s。

## 可选：仿真评测（复现论文指标）

按 InternNav 文档安装 Habitat-Sim/Lab，下载 R2R-CE / RxR-CE 数据集和 MP3D 场景，运行官方 eval 脚本，目标 R2R val-unseen SR ≈ 64。

## 可选：StreamVLN 对比

[InternRobotics/StreamVLN](https://github.com/InternRobotics/StreamVLN) 提供 `streamvln/http_realworld_server.py` 和真机权重 `mengwei0427/StreamVLN_Video_qwen_1_5_r2r_rxr_envdrop_scalevln_real_world`。把它的请求/回复格式适配成上面的协议后，就可以复用机器人端的 `vln_client`。
