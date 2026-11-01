# 车载电脑：摩方 MOREFINE M11（Intel N305）

| 项 | 值 |
|---|---|
| CPU | Intel Core i3-N305（8 核 8 线程，最高 3.8 GHz，TDP 约 15 W） |
| GPU | Intel UHD（32 EU），**无 CUDA** |
| 系统 | Ubuntu 22.04（jammy）→ ROS 2 Humble |
| 屏幕 | 7 寸触摸屏 |

## 定位

- 负责：传感器驱动、TF、HTTP 客户端、底层控制、安全停障、建图里程计（FAST-LIO 的 CPU 负载可以接受）。
- **不负责**：VLM 推理。7B 级模型放在 GPU 服务器上，经 Wi-Fi 用 HTTP 调用（参考 InternNav 真机部署方式）。

## 建议

- 电源：车载用 12 V→DC 输入（确认 M11 的输入规格），不要依赖平板内置电池长时间跑。
- 性能模式：`sudo apt install cpufrequtils && sudo cpufreq-set -r -g performance`。
- 网络：有线网口给 Mid-360（静态 IP），Wi-Fi 连 GPU 服务器，最好用 5 GHz 并且服务器有线接入路由器。
- 远程调试：`sudo apt install openssh-server`，用 VS Code Remote-SSH 开发。
- DDS：使用 CycloneDDS（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，`01_install_deps.sh` 已设置），固定 `ROS_DOMAIN_ID=42`。
- 图像传输：RViz 远程查看时使用 `compressed` 话题，避免占满 Wi-Fi。
