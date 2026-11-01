# 实车 bring-up 记录

## 2026-11-01（M11 `caoruixiang-M11`，Wi-Fi 192.168.8.174）

| 设备 | 接口 / 地址 | 结果 |
|---|---|---|
| RoboMaster S1 | USB RNDIS `enx…` 192.168.42.3 ↔ S1 192.168.42.2；SN `3JKDH2T0012Z1K`，固件 01.01.1150 | ✅ SDK `drive_wheels` / `drive_speed` 可驱动，ESC 回传正常 |
| WHEELTEC G60 | CH9102 1a86:55d4 → `/dev/ttyACM0`（CDC-ACM，不是 ttyUSB），`/dev/wheeltec_gps` 软链已存在 | ✅ NMEA 输出正常，室内无定位 |
| Livox Mid-360 | `enp1s0`，雷达 IP **192.168.1.156** | ✅ 点云 UDP 56300（~1750 pkt/s）、IMU 56400（~170 Hz） |
| USB 摄像头 | "USB 2.0 Camera 4K" eba4:1303 → `/dev/video0`（`/dev/vln_cam`） | ✅ 1280×720 MJPG 30 fps，画面正向（未倒置） |

### 注意

- Mid-360 当前配置的目的主机为 **192.168.1.100**（点云/IMU）和 **192.168.1.5**（日志/状态），而 M11 的 `enp1s0` 是 192.168.1.101。livox_ros_driver2 的 `MID360_config.json` 要把 `host_ip` 设成 192.168.1.101，雷达 IP 设成 192.168.1.156。
- `drive_wheels` 原始 ESC 转速中左侧两个电机符号相反（前进时 `[+,-,-,+]`），属正常现象。
- **架空时不要测试 `drive_speed` 的旋转（z）**：底盘用陀螺仪闭环控制偏航，架空时检测不到转动，会一直加速（实测约 1.5 s 升到约 470 rpm）。只在地面上测试旋转。
- 摄像头之前被设成手动曝光（exposure=13），画面几乎全黑。udev 规则会在插入时恢复 `auto_exposure=3`。
- `/dev/ttyACM1` 是 S1 智能中控的串口（DJI）。

### 已暂时关闭的另一个项目（camera-host，Hi3516 相机阵列）

| 项目 | 动作 |
|---|---|
| `~/.config/autostart/snap-gui.desktop` | 移到 `~/.camera-host-autostart-disabled/` |
| crontab `@reboot … sync_beacon.py` | 注释掉（原文件已备份） |
| `dnsmasq-openipc.service`、`snap-dhcp.service` | `systemctl disable --now` |
| `cam-lan.service`（设置 enp1s0 = 192.168.1.101） | **保留**，雷达要用这个网口 |

恢复：`~/.camera-host-autostart-disabled/restore.sh`
