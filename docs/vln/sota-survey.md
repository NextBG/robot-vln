# VLN SOTA 调研（截至 2026 年）

目标：在 **连续环境 VLN（VLN-CE，R2R/RxR）** 上选一个有开源真机部署代码的 SOTA 方法，部署到 S1 小车上。

| 方法 | 会议 | 输入 | R2R-CE SR / SPL | 开源 | 真机代码 | 备注 |
|---|---|---|---|---|---|---|
| **InternVLA-N1 (DualVLN)** | ICLR 2026 | RGB（System1 可选 RGB-D） | **64.3 / 58.5** | ✅ 代码 + 权重 | ✅ `scripts/realworld/` | **首选**。慢系统 VLM（Qwen2.5-VL 7B）出像素目标，快系统 DiT 出轨迹；RxR SR 61.4 |
| StreamVLN | ICRA 2026 | RGB | ~56.9 / 51.9 | ✅ | ✅ `http_realworld_server.py` + Go2 客户端，有 real-world 权重 | 流式 KV cache，延迟低，适合做对比基线 |
| JanusVLN | ICLR 2026 | RGB（隐式 3D） | 复现约 53 | ✅ | ❌ | 双隐式记忆 |
| AgentVLN | ECCV 2026 | RGB | — | ✅ | 部分 | Agent 式 |
| NaVid / Uni-NaVid / NaVILA | 2024–25 | RGB | 37–54 | ✅ | 部分 | 较早的视频 VLM 方法 |

数据来自各仓库 README；最新进展可以看 [awesome-embodied-vla-va-vln](https://github.com/jonyzhang2023/awesome-embodied-vla-va-vln)。

## 为什么选 InternVLA-N1 (DualVLN)

1. 在 R2R-CE / RxR-CE 的纯 RGB 设定下排名第一，并且有官方真机部署脚本（HTTP 服务端 + 客户端）。
2. 输出是**机体坐标系轨迹**或离散动作，和差速/麦轮底盘都容易对接。
3. 权重在 HuggingFace 上公开：`InternRobotics/InternVLA-N1-DualVLN`，另有 `InternVLA-N1-w-NavDP`、`InternVLA-N1-System2`。
4. 社区已有 Go2 部署经验可参考：[cmjang/InternNav-deploy](https://github.com/cmjang/InternNav-deploy)。

## 复现计划

1. **仿真对齐**：在 GPU 服务器上安装 InternNav + Habitat，用官方评测脚本在 R2R-CE val-unseen 上复现 SR≈64。
2. **真机开环**：机器人推流到服务器，只看输出，不驱动底盘，检查延迟和动作是否合理。
3. **真机闭环**：`vln_client` 执行轨迹，`safety_stop` 兜底，从室内短指令开始测试。
4. **对比实验**：同一套客户端协议接入 StreamVLN 的服务端（只需改服务端适配层）。

## 风险

- 训练数据的相机高度和视场角（Habitat 中约 1.25 m、HFOV 79°）与 S1 车载相机差别较大，可能需要抬高相机，或在真实数据上微调。
- 没有深度相机：使用 Mid-360 投影的稀疏深度或服务端单目深度估计，NavDP System1 的效果需要实测。
- GPU 服务器要求：7B VLM 推理建议 ≥24 GB 显存（RTX 3090/4090/A5000 及以上）。这是估计值，以官方说明为准。
