# VLN 部署架构

## 数据流

1. `usb_cam` → `/camera/image_raw`（640×480 RGB）
2. `lidar_depth_projector`：`/livox/lidar` + `/camera/camera_info` + TF(`camera_optical_frame` ← `livox_frame`) → `/vln/depth`（32FC1，米）
3. `vln_client`（默认约 2 Hz）：拿最新的 RGB（+深度）和 `/odom` 位姿 → HTTP `POST /eval_dual` → 解析回复
   - `discrete_action`：0 停止，1 前进 0.25 m，2 左转 15°，3 右转 15°（5 = 低头，由服务端处理）→ 转成世界坐标目标点 → 点到点控制器
   - `trajectory`：机体系 (x 前, y 左) 路点 → 按拍照时刻的位姿转到世界系 → pure pursuit
   - 输出 `/vln/cmd_vel`
4. `safety_stop`：`/vln/cmd_vel` → 限幅 + 前方障碍检测（Mid-360 点云）+ 看门狗 → `/cmd_vel` → `robomaster_ros`

## HTTP 协议（与 InternNav realworld 一致，额外支持 instruction 字段）

```
POST http://<server>:5801/eval_dual   multipart/form-data
  image: image/jpeg
  depth: image/png  (uint16, 值 = 米 × 10000)
  json : {"reset": true|false, "idx": <int>, "instruction": "<自然语言指令>"}
→ {"discrete_action": [1,1,2]}  或  {"trajectory": [[x,y],...], "pixel_goal": [u,v]}
```

## ROS 接口

| 话题 | 方向 | 类型 |
|---|---|---|
| `/vln/instruction` | 输入 | std_msgs/String，发布一次即开始新任务 |
| `/vln/stop` | 输入 | std_msgs/Empty |
| `/vln/status` | 输出 | std_msgs/String |
| `/vln/cmd_vel` → `/cmd_vel` | 输出 | geometry_msgs/Twist |

## 时延

N305 编码 JPEG 约 5 ms，Wi-Fi 上传 30–80 ms，服务器端 System2 约 0.3–1 s 一次（System1 更快）。所以客户端采用**异步请求 + 持续跟踪上一条轨迹**的方式，看门狗超时后停车。
