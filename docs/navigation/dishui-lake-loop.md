# 滴水湖环湖 GPS 循迹（凯悦嘉轩出发、回到凯悦嘉轩）

包：`ros2_ws/src/vln_robot_nav`；路线工具：`tools/amap_route.py`。

## 路线

- 起点和终点：上海临港凯悦嘉轩酒店（高德 POI GCJ-02 `121.933527,30.909469`，水芸北路 99 号）
- 路线：酒店 → 约 216 m 连接段 → 逆时针绕滴水湖环湖步道 → 原路返回酒店
- 全长 **9171 m**，每 5 m 一个点，共 1844 个点
- 文件：`routes/dishui_lake.csv`（WGS-84）、`routes/dishui_lake.geojson`

重新生成路线（只在开发机上运行；高德 Key 只能通过环境变量传入，**不要提交到仓库**）：

```bash
AMAP_KEY=xxxx python tools/amap_route.py --help   # 可用 --start 指定其他起点（GCJ-02）
```

高德返回的是 GCJ-02 坐标，脚本会转换成 WGS-84，和 G60 输出的坐标一致。

## 算法（`follower.py`）

状态机：`WAIT_FIX → ALIGN → FOLLOW → DONE`，另有 `HOLD`（暂停）和 `FAULT`（故障）。

- **ALIGN（对准）**：直行约 10 m，比较 GPS 位移方向和里程计位移方向，得到航向偏差。S1 的 IMU 没有磁力计，所以航向要靠这一步估计。之后每走 25 m 用低通滤波（增益 0.1）更新一次。
- **位置**：里程计的增量转到 ENU 坐标系，再用 GPS 修正（增益 0.25）。
- **控制**：纯追踪（pure pursuit），前视距离 4 m。目标点偏离超过 60° 时原地转向。
- **进度**：沿路线的里程 s 只增不减，搜索窗口为 [-15, +40] m，并截止在路线终点。这样往返重叠的连接段不会选错方向。
- **保护**：
  - 起点离路线超过 25 m，或横向偏差超过 12 m，进入 FAULT
  - 2.5 s 没有可用定位，进入 HOLD
  - 距离终点 3 m 以内判为 DONE
  - `safety_stop` 用雷达检测，0.45 m 内有障碍就停车

## 仿真（不需要硬件）

```bash
source ~/robot-vln/scripts/onboard/ros_env.sh
python3 -m pytest -p no:anyio ros2_ws/src/vln_robot_nav/test     # 7 个测试，含 2 次整圈仿真
ros2 launch vln_robot_nav gps_loop.launch.py sim:=true autostart:=true time_scale:=20
ros2 topic echo /nav/status
```

## 实地操作步骤

1. 先在空地上校准里程计比例：直线走 5 m，对比 `/odom` 显示的距离和尺量的实际距离。架空时测不出来。
2. 把机器人放到酒店门口的路线起点，**车头朝向连接段的方向**（朝湖边走）。
3. 在室外等 G60 定位。`ros2 topic echo /fix` 中 `status >= 0` 才表示已定位，冷启动通常需要 1–3 分钟。
4. 启动：

   ```bash
   ros2 launch vln_robot_nav gps_loop.launch.py max_v:=0.6
   ros2 service call /nav/start std_srvs/srv/Trigger {}
   ```

5. 监控：`ros2 topic echo /nav/status`（JSON，包含状态、已走里程 s、横向偏差、航向偏差）。
6. 暂停用 `ros2 service call /nav/stop std_srvs/srv/Trigger {}`。换完电池后继续：

   ```bash
   ros2 launch vln_robot_nav gps_loop.launch.py resume:=true
   ```

   然后再调用一次 `/nav/start`。进度保存在 `~/.ros/gps_route_progress.json`。

## 注意事项

- **续航**：按 0.6 m/s 走完 9.2 km 约需 4.3 小时，S1 电池约 35 分钟，至少要换 7 次电池或外接电源。M11 也要考虑供电。
- **定位精度**：G60 约 2.5 m（CEP）。树荫和高楼附近误差会变大，环湖步道的宽度要能容纳这个误差。
- **航向**：开始时必须先直行 ALIGN。如果车头没对准路线，前 10 m 会偏离路线。
- 全程需要有人跟随，并能随时调用 `/nav/stop` 或直接断电。
- 不要在架空状态下测试原地旋转：陀螺仪闭环会一直加速。
