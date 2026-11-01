# RoboMaster S1 底盘：如何控制底层轮子旋转

## 1. 硬件结构

S1 内部有两个控制器：

- **运动控制器（主控）**：直接驱动 4 个 M3508 类无刷电机（通过电调），挂在内部 **CAN 总线**上。
- **智能中控（Intelligent Controller）**：一块 Android 板，负责 Wi-Fi、相机、App。它通过 CAN 与运动控制器通信，对外提供 RoboMaster SDK（文本/二进制协议）。

所以"控制轮子"有两条路：

| 方案 | 连接 | 延迟/频率 | 难度 | 推荐 |
|---|---|---|---|---|
| A. SDK（经智能中控） | M11 USB 线 → 智能中控 micro-USB（RNDIS，机器人 IP 192.168.42.2）或 Wi-Fi | ~20–50 Hz | 低 | ✅ 先用这个 |
| B. 内部 CAN 直连 | USB-CAN 适配器（SocketCAN，如 CANable/candleLight）接到 S1 CAN 总线 | 高，可到电机级 | 中高（需接线） | 需要更低延迟或拆掉智能中控时用 |

> DJI 官方 SDK 只正式支持 EP。S1 需要先在 App 里解锁 SDK 模式（社区做法），或者直接用 `jeguzzi` 的 SDK fork 加 `robomaster_ros`，作者明确支持 S1。

## 2. 方案 A：RoboMaster SDK / robomaster_ros

### 2.1 SDK 原生调用（无 ROS，见 `tools/s1_wheel_test.py`）

```python
from robomaster import robot
ep = robot.Robot(); ep.initialize(conn_type="rndis")
ch = ep.chassis
# 单独控制每个轮子转速（rpm，范围约 ±1000）
# w1 右前, w2 左前, w3 左后, w4 右后
ch.drive_wheels(w1=60, w2=60, w3=60, w4=60, timeout=1)
# 车体速度：x 前进 m/s（±3.5），y 左移 m/s（±3.5），z 角速度 deg/s（±600）
ch.drive_speed(x=0.2, y=0, z=0, timeout=1)
# 订阅电调反馈：转速、编码器角度、时间戳、状态
ch.sub_esc(freq=10, callback=lambda info: print(info))
```

轮序、方向和取值范围请以 `s1_wheel_test.py` 的实测结果为准，测试前**把车架空**。

### 2.2 ROS 2：`jeguzzi/robomaster_ros`

`scripts/onboard/05_build_ws.sh` 会把它拉到 `ros2_ws/src/third_party/`。SDK fork 由 `01_install_deps.sh` 用 pip 安装（需要 `libopus-dev`）。

| 话题 | 类型 | 说明 |
|---|---|---|
| `cmd_vel` | geometry_msgs/Twist | 车体速度（麦轮可以 `linear.y` 横移） |
| `cmd_wheels` | robomaster_msgs/WheelSpeeds | **单轮线速度** m/s：front_left / front_right / rear_left / rear_right |
| `odom` | nav_msgs/Odometry | 轮速里程计 |
| `imu` | sensor_msgs/Imu | 中控 IMU |

```bash
ros2 launch robomaster_ros s1.launch conn_type:=rndis serial_number:=<SN> video:=-1 audio:=-1
ros2 topic pub -r 10 /cmd_wheels robomaster_msgs/WheelSpeeds "{front_left: 0.2, front_right: 0.2, rear_left: 0.2, rear_right: 0.2}"
```

本仓库的 `vln_robot_bringup/launch/robot.launch.py` 已经封装好（`use_chassis:=true`）。

## 3. 方案 B：内部 CAN（可选）

- [proroklab/robomaster_sdk_can](https://github.com/proroklab/robomaster_sdk_can)：C++ 库，直接在 S1/EP 的 CAN 总线上和运动控制器通信。`send_workmode(1)` 之后用 `send_wheel_speed(...)` 控制 4 轮。
- [proroklab/robomaster_ros2_can](https://github.com/proroklab/robomaster_ros2_can)：ROS 2 bridge（`ros2 launch robomaster_ros2_can bridge.launch.py`）。
- [RoboMasterS1Challenge/robomaster_s1_can_hack](https://github.com/RoboMasterS1Challenge/robomaster_s1_can_hack)：CAN 协议逆向资料。

```bash
WITH_CAN=1 ./scripts/onboard/05_build_ws.sh
sudo ip link set can0 up type can bitrate 1000000
candump can0           # 先确认能看到总线报文
```

> CAN 方案需要从智能中控/主控之间的总线引出 CAN_H/CAN_L/GND，注意不要接错电源线。

## 4. 麦克纳姆轮运动学（参考）

轮半径 r≈0.05 m（需实测），半轴距 lx（前后）、ly（左右）：

```
v_fl = vx - vy - (lx+ly)·wz
v_fr = vx + vy + (lx+ly)·wz
v_rl = vx + vy - (lx+ly)·wz
v_rr = vx - vy + (lx+ly)·wz
rpm  = v / (2π r) · 60
```

`robomaster_ros` 的参数 `chassis.twist_to_wheel_speeds: true` 会在驱动内部做这个换算。

## 5. 常见问题

- RNDIS 连不上：`ip a` 里应出现 `usb0`/`enx...` 并获得 192.168.42.x 地址，用 `ping 192.168.42.2` 检查。
- SDK 报找不到机器人：确认 S1 固件已开启 SDK 模式，并且 `serial_number` 正确（launch 参数，可留空自动发现）。
- 电量低时电机限功率，速度会变慢。
