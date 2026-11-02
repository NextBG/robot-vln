# WHEELTEC G60 GNSS 模块

厂家原始资料（手册 PDF、ROS2/Python 例程、CAD）随模块附送，或向 WHEELTEC 索取。这些资料有版权，本仓库不收录；本地可放在 `vendor/wheeltec_g60/`（已加入 .gitignore）。

## 1. 参数（摘自厂家手册）

| 项 | 值 |
|---|---|
| 芯片 | 中科微 ATGM336H-5N（GPS + BDS 等多系统） |
| 输出 | NMEA0183（$GNGGA / $GNRMC / ...） |
| 波特率 | 出厂 **9600**；在 GnssToolKit 里恢复出厂后为 115200 |
| 更新率 | 默认 1 Hz，最高 10 Hz |
| 定位精度 | < 2.5 m（单点，开阔环境） |
| 供电/接口 | 5 V，Type-C（USB 转串口 CH9102F，VID:PID = 1a86:55d4） |
| 天线 | MCX 接口有源天线 |

## 2. Linux 下的设备名和 udev（重要坑）

- Ubuntu 22.04 自带的 `cdc_acm` 驱动会把 CH9102F 识别成 `/dev/ttyACM*`；装了厂家 CH343 驱动后则是 `/dev/ttyCH343USB*`。
- 厂家 `wheeltec_udev.sh` 里 `/dev/wheeltec_gps` 规则写的是老款 CP2102（10c4:ea60），CH9102 被映射成 `/dev/wheeltec_gnss`，但厂家 launch 用的却是 `/dev/wheeltec_gps`。**这个规则对新模块无效。**
- 本仓库的 [`scripts/onboard/99-vln-robot.rules`](../../scripts/onboard/99-vln-robot.rules) 三种情况都会映射成 `/dev/wheeltec_gps`，前提是 **USB 芯片序列号为 `0005`**。厂家手册的做法是：在 Windows 上用 `CH34xSerCfg.exe` 把序列号改成 0005（见手册"udev 绑定"一节）。

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
sudo cat /dev/wheeltec_gps        # 应看到 $GNGGA... 语句
```

## 3. ROS 2

使用官方 apt 包（与厂家例程中的 `nmea_navsat_driver` 同源）：

```bash
sudo apt install ros-humble-nmea-navsat-driver ros-humble-nmea-msgs
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/wheeltec_gps -p baud:=9600 -p frame_id:=gps_link
ros2 topic echo /fix
```

参数见 `ros2_ws/src/vln_robot_bringup/config/gnss.yaml`，`robot.launch.py` 中 `use_gnss:=true` 会启动它。
厂家例程里的 `wheeltec_gps_path` 包（GPS 轨迹转 Path，在 RViz 显示）可选用，需要时自行放到 `ros2_ws/src/third_party/`（已 gitignore）。

## 4. 提升性能

- **10 Hz 输出**：用 GnssToolKit（Windows）修改输出频率并保存，同时把波特率提高到 115200，然后修改 `gnss.yaml`。
- **AGNSS 辅助定位**（冷启动更快）：服务器 `www.gnss-aide.com:2621`，账号 `agnss_user@wheeltec.net` / 密码 `wheeltec`（厂家公开账号，见手册）。
- 天线要放在车顶无遮挡的位置，远离雷达和电机电源线。

## 5. 未纳入仓库的厂家文件（体积大 / Windows 工具 / ROS1）

仍保存在本地 `C:\baidunetdiskdownload\WHEELTEC G60模块附送资料\`：

- GnssToolKit 上位机、串口助手（Windows 工具，各约 25 MB）
- STM32 例程（两个 zip）
- ROS1 例程 zip
