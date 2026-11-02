# CityWalker 视觉局部规划

[CityWalker](https://github.com/ai4ce/CityWalker)（CVPR 2025，Apache-2.0）用 2000 小时城市步行视频训练：输入最近 5 帧相机画面（1 Hz）、过去 5 个位置和一个目标点，输出未来 5 个路径点（约 1 s 一个）和“已到达”概率。我们用它代替“直勾勾朝下一个路径点走”的 GPS 纯追踪：

```
GPS 路线 (task / follower) ──前方 cw_target_ahead=12 m 的路线点──┐
相机 /camera/image_raw ── 1 Hz 5 帧 + 里程计历史位置 ──────────────┴─> GPU: citywalker_server.py (5802)
                                                                         │ 5 个局部路径点（车体系，米）
safety_stop <─ /nav/cmd_vel <─ 纯追踪（前视 1.5 m）<───────────────────┘
```

- GPS 路线仍负责进度、到点判定、偏离过大故障停车（FAULT），CityWalker 只决定“这几米怎么走”。
- 规划超过 3 s 没更新（服务器断开、隧道断开）就自动退回 GPS 纯追踪；模型连续 10 s 要求原地不动时也交还给路线。
- `safety_stop` 依旧是最后一道保护（前方障碍停车、限速、急停）。

## 坐标约定（已对照官方代码核对）

预训练数据（`data/citywalk_feat_dataset.py`）的位姿来自视觉 SLAM，取 `[x, z]`：**x 向右、z 向前**，以当前帧为原点；所有位置除以 `step_scale`（该视频 1 Hz 下的平均步长）。服务器接口用车体系（x 前、y 左、米），内部转换：`model = (-y, x) / step_scale`，输出 `wp * step_scale` 再转回车体系。图像：缩放到 640 宽，居中裁成 360×640（模型内部再裁 350×630），ImageNet 归一化在模型内部完成。机器人上 `step_scale` 默认取 `0.85 × max_v`（1 s 内的平均行进距离）。

## GPU 服务器

```bash
bash /root/robot-vln/server/setup_citywalker.sh      # CityWalker + dinov2 源码、权重 CityWalker_2000hr.ckpt
cd /root && nohup /root/miniconda3/envs/internnav/bin/python /root/robot-vln/server/citywalker_server.py \
  --citywalker_root /root/CityWalker --dinov2_root /root/dinov2 \
  --ckpt /root/CityWalker/checkpoints/CityWalker_2000hr.ckpt --port 5802 > /root/citywalker.log 2>&1 &
curl -s 127.0.0.1:5802/health
```

- 不需要 pytorch-lightning / xformers：直接读 checkpoint 的 `state_dict`，DINOv2 结构从本地 hub 源码构建（权重在 checkpoint 里）。
- RTX 4090 上单次推理约 50 ms，显存约 1 GB，可以和 InternVLA-N1（5801）同时运行。
- 接口：`POST /citywalker`，multipart `image0..image4`（JPEG，旧→新）+ `json={"hist": [[x,y]×5], "target": [x,y], "step_scale": s}`，返回 `{"waypoints": [[x,y]×5], "arrive_prob", "infer_ms"}`。

## 机器人

网页“GPS 导航任务”里把 **局部规划** 选成 “CityWalker 视觉规划（GPU）”，预览后启动。隧道需要同时转发 5802（见 [dashboard.md](../web/dashboard.md)）。面板显示规划状态、往返耗时、到达概率；雷达俯视图中品红色折线是当前局部轨迹。

`gps_waypoint_follower` 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `planner` | gps | `gps` / `citywalker` |
| `cw_server_url` | `http://127.0.0.1:5802/citywalker` | |
| `cw_target_ahead` | 12.0 | 目标点取路线前方多少米（训练时目标距离约 10–50 步） |
| `cw_step_scale` | 0 → `0.85*max_v` | 米 / 步（1 s） |
| `cw_lookahead` | 1.5 | 在局部轨迹上的前视距离 |
| `cw_plan_timeout` | 3.0 | 规划过期时间，过期退回 GPS 追踪 |
| `cw_max_hold` | 10.0 | 模型要求停下最多遵守多久 |

## 注意

- 训练数据是行人视角（约 1.5 m 高）、步速约 1.3 m/s；S1 相机离地约 0.3 m、速度 0.3 m/s，属于域外，需要先在户外**低速、有人跟随**验证，必要时用官方 `fine_tune.py`（teleop 数据）微调。
- 室内没有 GPS 时导航不会进入 FOLLOW，也就不会请求 CityWalker。
