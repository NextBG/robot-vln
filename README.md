# robot-vln

基于 **ROS 2 Humble** 的室外/室内移动机器人平台，目标是在真机上复现当前 SOTA 的视觉-语言导航（VLN）方法。
**首选复现目标：InternVLA-N1 (DualVLN)**，详细对比见 [docs/vln/sota-survey.md](docs/vln/sota-survey.md)。

> 当前阶段：**环境准备**。车载电脑是 摩方 M11（Intel N305，Ubuntu 22.04）。所有安装脚本都在车载电脑上运行，**不要在开发机上运行**。

## 硬件

| 模块 | 型号 | 接口 | ROS 2 驱动 | 文档 |
|---|---|---|---|---|
| 底盘 | DJI RoboMaster S1（麦克纳姆轮） | micro-USB RNDIS（智能中控）/ 可选内部 CAN | `jeguzzi/robomaster_ros`；可选 `proroklab/robomaster_ros2_can` | [robomaster-s1-chassis.md](docs/hardware/robomaster-s1-chassis.md) |
| GNSS | WHEELTEC G60（ATGM336H-5N，NMEA） | USB Type-C（CH9102F） | `nmea_navsat_driver` | [wheeltec-g60-gps.md](docs/hardware/wheeltec-g60-gps.md) |
| 相机 | 通用 USB UVC 摄像头 | USB | `usb_cam` | [usb-webcam.md](docs/hardware/usb-webcam.md) |
| 激光雷达 | Livox Mid-360（非 S 版） | 千兆以太网，9–27 V 供电 | `livox_ros_driver2` + Livox-SDK2 | [livox-mid360.md](docs/hardware/livox-mid360.md) |
| 车载电脑 | 摩方 M11 7 寸平板 mini PC（N305，无独显） | — | Ubuntu 22.04 + ROS 2 Humble | [m11-n305-onboard-pc.md](docs/hardware/m11-n305-onboard-pc.md) |
| 推理服务器 | 任意 NVIDIA GPU 主机（建议 ≥24 GB 显存） | Wi-Fi/局域网 HTTP | InternNav | [gpu-server.md](docs/setup/gpu-server.md) |

## 系统架构

```
 ┌──────────────── 机器人 (M11 / N305, ROS 2 Humble) ─────────────────┐        ┌──── GPU 服务器 ────┐
 │ usb_cam ──/camera/image_raw──┐                                       │        │ InternVLA-N1       │
 │ livox ──/livox/lidar──► lidar_depth_projector ──/vln/depth──┐        │  HTTP  │ (System2 VLM +     │
 │                               └───────────────► vln_client ─┼────────┼──────► │  System1 planner)  │
 │ robomaster_ros ──/odom──────────────────────────► vln_client ◄────────┼─────── │ discrete_action /  │
 │                                                     │ /vln/cmd_vel    │        │ trajectory         │
 │ robomaster_ros ◄── /cmd_vel ── safety_stop ◄────────┘  (+/livox/lidar)│        └────────────────────┘
 │ nmea_navsat_driver ──/fix (G60)                                      │
 └──────────────────────────────────────────────────────────────────────┘
```

N305 跑不动 7B 级 VLM，所以模型放在 GPU 服务器上。机器人只负责采集传感器数据、发 HTTP 请求和执行底层控制，这和 InternNav 官方真机部署方式一致。

## 目录结构

```
docs/
  hardware/      各硬件资料（底层控制、接线、参数、坑）
  vln/           SOTA 调研、部署架构
  setup/         车载电脑 / GPU 服务器 安装指南
vendor/
  wheeltec_g60/  厂家资料（手册 PDF、ROS2/Python 例程、CAD）
ros2_ws/
  robot.repos    第三方 ROS2 源码（vcs import）
  can.repos      可选：S1 内部 CAN 方案
  src/vln_robot_bringup/   整车启动（底盘+相机+雷达+GNSS+TF）
  src/vln_robot_client/    VLN HTTP 客户端、雷达->深度投影、安全停障
  src/third_party/wheeltec_gps_path/  厂家 GPS 轨迹包
scripts/onboard/ 车载电脑一键安装脚本（00~05）
server/          GPU 端 InternVLA-N1 HTTP 服务封装
tools/           独立测试脚本（S1 轮速测试等）
```

## 快速开始（在 M11 上）

```bash
git clone https://github.com/NextBG/robot-vln.git ~/robot-vln && cd ~/robot-vln
LIDAR_IFACE=enp1s0 USE_CN_MIRROR=1 ./scripts/onboard/setup_all.sh   # 网卡名用 `ip -br link` 查看
# 重新登录后：
ros2 launch vln_robot_bringup robot.launch.py serial_number:=<S1序列号>
ros2 launch vln_robot_client vln.launch.py server_url:=http://<GPU服务器IP>:5801/eval_dual
ros2 topic pub --once /vln/instruction std_msgs/String "data: 'Go forward and stop at the door.'"
```

GPU 服务器端见 [docs/setup/gpu-server.md](docs/setup/gpu-server.md)。

GPS 环湖循迹（滴水湖，凯悦嘉轩往返）见 [docs/navigation/dishui-lake-loop.md](docs/navigation/dishui-lake-loop.md)。

## 需要你填写的占位项

- [x] `ros2_ws/src/vln_robot_bringup/config/MID360_config.json`：雷达 192.168.1.156，主机 192.168.1.101
- [x] `config/camera_info.yaml`：已标定（2026-11-02，fx 689.7 fy 691.1 cx 324.4 cy 250.0，RMS 0.12 px）
- [ ] `config/extrinsics.yaml`：相机、雷达、GPS 相对 `base_link` 的安装位姿
- [ ] `scripts/onboard/99-vln-robot.rules`：摄像头 VID/PID；G60 的 CH9102 序列号改为 `0005`
- [x] GPU 服务器 `--intrinsic 689.7 691.1 324.4 250.0`

## 路线图

1. ✅ 资料整理、仓库与环境脚本
2. ✅ 在 M11 上装机，逐个打通传感器（`scripts/onboard/check_hardware.sh`）
3. ⬜ 相机标定 + 外参测量；S1 底盘遥控测试（`tools/s1_wheel_test.py`、teleop）
4. 🔶 GPU 服务器部署 InternVLA-N1（✅ 推理服务已跑通，约 0.3 s/步），先跑仿真评测（VLN-CE R2R）对齐论文指标
5. ⬜ 真机闭环：`vln_client` ↔ 服务器，室内短指令测试
6. 🔶 GPS 环湖循迹（vln_robot_nav，已完成仿真，待实地测试）
7. ⬜ 可选：FAST-LIO2 里程计替代轮速计；StreamVLN / JanusVLN 对比实验
