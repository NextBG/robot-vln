# Livox Mid-360（非 S 版）

## 1. 关键参数

| 项 | 值 |
|---|---|
| FOV | 水平 360°，垂直 -7°~52°（共 59°） |
| 量程 | 40 m @10% 反射率，近处盲区约 0.1 m |
| 点频 | 200,000 点/秒，10 Hz |
| 内置 IMU | ICM40609（6 轴） |
| 接口 | 千兆以太网（UDP） |
| 供电 | **9–27 V DC**，典型功耗约 6.5 W，启动峰值更高 |

> 规格以 Livox 官网规格书为准。"Mid-360S"是另一个型号，不要混用配置。

## 2. 接线和供电

- 官方转接线（M12 航插 → 网口 + 电源 + 同步线）。
- S1 电池输出电压不稳定且难以取电，建议给雷达**单独供电**：12 V 锂电池（或 24 V 降压到 12 V）+ DC-DC 稳压。
- M11 的网口接雷达。M11 走 Wi-Fi 连 GPU 服务器。

## 3. 网络

- 雷达出厂静态 IP：**192.168.1.1XX**，XX = 序列号**最后两位**。
- 主机需要在同一网段：本仓库用 **192.168.1.101/24**（M11 `enp1s0`，由已有的 `cam-lan` 连接配置）（`scripts/onboard/04_network_mid360.sh`）。
- `ros2_ws/src/vln_robot_bringup/config/MID360_config.json` 中的 `host_net_info.*_ip` = 192.168.1.101，`lidar_configs[0].ip` = 192.168.1.156（本车雷达）。

## 4. 驱动

```bash
./scripts/onboard/02_install_livox_sdk2.sh   # Livox-SDK2 → /usr/local
./scripts/onboard/05_build_ws.sh             # 拉 livox_ros_driver2 并用 ROS_EDITION=ROS2 编译
```

| 参数 | 含义 |
|---|---|
| `xfer_format` | 0 = `sensor_msgs/PointCloud2`（默认，RViz 可直接看）；1 = `livox_ros_driver2/CustomMsg`（FAST-LIO 需要） |
| `publish_freq` | 10 Hz |
| `frame_id` | `livox_frame` |

话题为 `/livox/lidar` 和 `/livox/imu`。

## 5. 在本项目中的作用

1. **投影深度图**：`lidar_depth_projector` 把点云投影到相机平面，生成 `/vln/depth`，给需要 RGB-D 的 System1（NavDP）使用。
2. **安全停障**：`safety_stop` 在车前方检测障碍物，拦截 `/vln/cmd_vel`。
3. 后续可以用 FAST-LIO2 / Point-LIO 做激光惯性里程计，代替 S1 轮速计。
