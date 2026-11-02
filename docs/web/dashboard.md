# 机器人监控 / 遥控网页（vln_robot_web）

包：`ros2_ws/src/vln_robot_web`。车载电脑上跑一个纯 Python HTTP 服务（端口 8080）。用手机或电脑浏览器打开 `http://<机器人IP>:8080/` 即可，不需要额外的前端依赖（Leaflet 已随包提供）。

## 启动

```bash
source ~/robot-vln/scripts/onboard/ros_env.sh && source ~/robot-vln/ros2_ws/install/setup.bash
# 带 GPU 服务器隧道（VLN 需要）：
ros2 launch vln_robot_web web.launch.py \
  vln_tunnel:="ssh -N -o BatchMode=yes -o ServerAliveInterval=15 -o ExitOnForwardFailure=yes -L 5801:127.0.0.1:5801 -p <端口> root@<GPU服务器>"
# 开机自启（systemd 用户服务）：
bash scripts/onboard/install_web_service.sh
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `with_robot` | true | 由网页节点托管各驱动进程（底盘、相机、雷达、GNSS、TF） |
| `use_gnss` | true | 是否启动 G60 |
| `port` | 8080 | HTTP 端口 |
| `max_v` / `max_w` | 0.3 / 0.8 | 遥控和 safety_stop 的限速，单位分别为 m/s 和 rad/s |
| `vln_server_url` | `http://127.0.0.1:5801/eval_dual` | InternVLA-N1 服务地址 |
| `vln_tunnel` | 空 | 连到 GPU 服务器的 SSH 隧道命令，由网页节点托管，断开后自动重连 |
| `vln_use_depth` | true | VLN 请求是否附带雷达深度 |

默认**不需要访问令牌**，适合在局域网或移动 Wi-Fi 下使用。如果要暴露到公网，启动时加 `-p require_token:=true`，令牌保存在 `~/.config/vln_robot_web/token`。

## 功能

- **状态**
  - 各话题的实时频率：相机、雷达、里程计、GPS、导航
  - S1 连接状态、电池、急停（E-stop）
  - GPU 服务器是否在线、模型状态
  - 状态通过 SSE 推送，每 0.5 s 一次
- **地图**
  - 默认“卫星 + 路网”（高德卫星图 style=6，叠加透明路网 style=8）。也可以切换到高德街道、纯卫星或 OSM。
  - GPS 坐标在前端从 WGS-84 转成 GCJ-02（高德坐标系）。
  - 路线由**路径点**组成，相邻点之间用直线连接，鼠标悬停可以看到编号。
  - **下一路径点**用品红色标出：机器人到它之间画一条虚线，到达判定半径（`wp_radius`）画成半透明圆。已经过的路径点显示为灰色。
  - 导航节点没有运行时，显示 `routes/dishui_lake.csv` 里的路线，并按最近的线段估计下一路径点。
- **激光雷达俯视**：车体坐标系下 0.05–1.5 m 高度的点，按 8 cm 网格取每格最高点，颜色表示高度（蓝=低，如路沿 → 青 → 绿 → 黄 → 红=1.5 m 及以上），左下角有色标。
- **遥控**
  - 点 🎮 按钮或按 `T` 键切换遥控开关。
  - 开启后用 W/S 前进后退、A/D 转向，或拖动屏幕摇杆；按空格立即停车。
  - 速度指令走 `/vln/cmd_vel → safety_stop → /cmd_vel`，所以遥控时避障和限速仍然生效。
  - 0.4 s 没有收到新指令（死人开关），或者网络断开，就自动停车。
  - 手动驾驶会取消正在执行的 VLN 任务。
- **急停（E-stop）**：急停状态锁存在 `/safety/estop`，触发时会同时停止 VLN。
- **驱动进程 / 重连**
  - 每个驱动（chassis、camera、lidar、gnss、tf）、GPU 隧道和 VLN 客户端都是独立的进程组，崩溃后会自动重启。
  - 换电池后如果有模块掉线，点“重连”或“全部重连”即可，不用重启整个系统。
  - 各模块日志在 `~/.ros/vln_robot_web/<模块>.log`。
- **VLN 任务**
  - 输入指令后发送，发送前需要二次确认。如果 `vln_client` 还没启动，会自动启动。
  - 面板显示任务状态（空闲 / 等待图像 / 请求中 / 执行中 / 完成 / 错误）、步数、模型回复和请求往返耗时。
- **GPS 导航任务**（不再写死起终点）
  - **方式**：
    - 沿已有路线：从 `vln_robot_nav/share/routes/*.csv` 或 `~/.ros/vln_robot_web/routes/*.csv` 中选路线，起点/终点吸附到路线上，可选“反向”。来回重叠的路段（如滴水湖的出发连接段）会选最短的那一段。
    - 手动点选：起点 → 途经点… → 终点，相邻点之间走直线。
    - 高德步行规划：起点（默认当前位置）到终点，调用高德 Web 服务步行接口。Key 放在机器人的 `~/.config/vln_robot_web/amap_key`（或环境变量 `AMAP_KEY`），**不要提交到仓库**。
  - 可选“闭环”（回到起点）、到点判定半径（1–30 m；终点仍按 ≤3 m 判定到达）、速度（不超过 `max_v`）、路径点密度：“路径点间距”（2–100 m，直路上每隔多远一个点）和“弯道容差”（0.2–5 m，越小弯道点越密）。它们以 `wp_max_spacing` / `wp_simplify_tol` 参数传给跟随器，所以实际路径点与预览一致。
  - 点“选起点 / 选终点 / 加途经点”，再在地图上点击选点（自动把 GCJ-02 坐标转回 WGS-84）。
  - **预览**：橙色折线 + 编号路径点 + 半透明的判定半径圆，同时显示总长、点数和预计用时。
  - **启动 / 暂停 / 继续 / 结束任务**：启动前需要确认。任务保存在 `~/.ros/vln_robot_web/nav_task.{json,csv}`，进度保存在 `nav_progress.json`，网页重启或换电池后点“继续”会从上次的进度接着走。
  - `gps_waypoint_follower` 作为 `nav` 模块由网页托管：启动时总是暂停状态，就绪后网页再调用 `/nav/start`；崩溃后自动重启，但也只恢复到暂停状态，不会自己动起来。
  - 遥控、发送 VLN 指令或急停都会自动暂停导航。到达终点（DONE）或故障（FAULT）后停车。

## HTTP API

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/api/state` | 当前状态 JSON |
| GET | `/api/events` | SSE 状态流 |
| GET | `/api/track` | 已走过的 GPS 轨迹 |
| GET | `/api/route` | `{radius, source: nav\|task\|file, closed, points: [[lat, lon], …]}` |
| GET | `/api/nav/routes` | 可选基础路线、高德是否可用、已保存任务 |
| GET | `/api/nav/base?name=` | 基础路线（简化后的路径点） |
| POST | `/api/nav/preview` | `{"task": {mode, base, start, end, points, reverse, loop, radius, max_v, spacing}}` → 路径点、长度、预计用时 |
| POST | `/api/nav/start` | 带 `task` 为新任务（从头开始），不带为继续 |
| POST | `/api/nav/pause` | 暂停导航 |
| POST | `/api/nav/cancel` | 结束任务（停止导航节点） |
| POST | `/api/cmd` | 遥控速度 `{"vx","vy","wz"}` |
| POST | `/api/stop` | 立即停车 |
| POST | `/api/estop` | `{"on": true\|false}` |
| POST | `/api/module` | `{"name": chassis\|camera\|lidar\|gnss\|tf\|vln\|tunnel\|nav\|all, "action": restart\|start\|stop}` |
| POST | `/api/vln` | `{"instruction": "..."}` |
| POST | `/api/vln/stop` | 停止 VLN 任务 |

## 实现要点和坑

- **抱起来轮子会转**：robomaster_ros 默认用固件的底盘速度模式。这个模式会用陀螺仪做航向闭环，车一倾斜或被抬起，固件就会转轮子去纠正航向。现在 `robot.launch.py` 改成直接控制轮速（`chassis_twist_to_wheel_speeds:=true`），并设置 `chassis_timeout:=0.5`。
- **子进程管理**（`supervisor.py`）：
  - `PR_SET_PDEATHSIG` 绑定的是**创建子进程的那个线程**：这个线程一退出，子进程就会收到信号。所以子进程只能由常驻的 `_watch` 线程创建；如果由处理请求的线程创建，请求处理完子进程就会被误杀。
  - 子进程启动时把 SIGINT 和 SIGTERM 恢复为默认处理，因为用 `&` 在后台启动的进程会忽略 SIGINT。
  - 停止进程组的顺序：先发 SIGINT 等 6 s，再发 SIGTERM 等 3 s，最后 SIGKILL。
- **雷达外参**：Mid-360 的安装方向和车头差 90°（雷达逆时针转 90° 才是车头），所以 `extrinsics.yaml` 里 livox 的 yaw 设为 `-1.5708`。
