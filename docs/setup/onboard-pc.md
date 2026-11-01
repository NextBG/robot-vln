# 车载电脑安装指南（M11 / Ubuntu 22.04）

> 所有步骤都在 **M11 上**执行。

```bash
sudo apt install -y git
git clone https://github.com/NextBG/robot-vln.git ~/robot-vln && cd ~/robot-vln
chmod +x scripts/onboard/*.sh
```

| 步骤 | 脚本 | 作用 |
|---|---|---|
| 0 | `00_install_ros2_humble.sh` | ROS 2 Humble Desktop、rosdep、colcon（`USE_CN_MIRROR=1` 使用清华源） |
| 1 | `01_install_deps.sh` | usb_cam、nmea_navsat_driver、cv_bridge、PCL、CycloneDDS、RoboMaster SDK（jeguzzi fork） |
| 2 | `02_install_livox_sdk2.sh` | 编译安装 Livox-SDK2 |
| 3 | `03_udev.sh` | `/dev/wheeltec_gps`、`/dev/vln_cam` |
| 4 | `04_network_mid360.sh` | 雷达网口静态 IP 192.168.1.50（需要 `LIDAR_IFACE=`） |
| 5 | `05_build_ws.sh` | `vcs import` 第三方源码 + `colcon build`（`WITH_CAN=1` 同时拉取 CAN 方案） |
| ✓ | `check_hardware.sh` | 硬件自检 |

一键执行：`LIDAR_IFACE=enp1s0 USE_CN_MIRROR=1 ./scripts/onboard/setup_all.sh`，完成后**重新登录**一次（dialout 组权限才会生效）。

## 逐个测试传感器

```bash
source ~/robot-vln/ros2_ws/install/setup.bash
ros2 launch vln_robot_bringup robot.launch.py use_chassis:=false use_lidar:=false use_gnss:=false   # 只开相机
ros2 launch vln_robot_bringup robot.launch.py use_camera:=false use_chassis:=false use_gnss:=false  # 只开雷达
ros2 launch vln_robot_bringup robot.launch.py serial_number:=<S1 SN>                              # 全部
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # 架空后测试底盘
python3 tools/s1_wheel_test.py --rpm 60                    # 不用 ROS，直接测试单轮
```

## 国内网络

- ROS apt：`USE_CN_MIRROR=1`
- rosdep 失败时：`pip install rosdepc && sudo rosdepc init && rosdepc update`
- GitHub 访问慢：配置代理，或者手动把 `robot.repos` 里的地址换成镜像
