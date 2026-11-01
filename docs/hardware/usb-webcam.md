# 通用 USB 摄像头（UVC）

VLN 模型只需要单目 RGB（InternVLA-N1 的训练数据接近 640×480、HFOV 约 79°）。建议选：

- 支持 **MJPEG 640×480 @30fps**（N305 用 YUYV 带宽不够时也能保证帧率）
- 水平视场角 **70°–90°**，**定焦**，最好能关闭自动对焦
- 安装高度尽量接近人眼或训练数据的相机高度；可以略微向下倾斜

## 1. 查看设备

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
lsusb   # 记下 VID:PID，填进 scripts/onboard/99-vln-robot.rules，生成 /dev/vln_cam
```

## 2. ROS 2 驱动：wide_cam_node（默认，最宽视场）

本机摄像头（"USB 2.0 Camera 4K"，eba4:1303）**只有 4000×3000 模式使用整块传感器**。其他 MJPG 模式（2592×1944、1920×1080、1280×720、640×480 等）都是从中间裁出来的：4:3 模式只保留约 72%×77% 的画面，16:9 模式约 95%×77%。所以：

| 模式 | 水平视场 | 垂直视场 | 备注 |
|---|---|---|---|
| 4000×3000 缩放到 640×480（默认） | **65.9°** | **51.8°** | `wide_cam_node`，约 9–15 fps，占 1 个 CPU 核 |
| 640×480 原生（裁切） | 49.8° | 38.3° | `usb_cam`，`camera_driver:=usb_cam` |

usb_cam 会把 1200 万像素的帧完整解码，N305 跑不动。所以 `vln_robot_bringup/wide_cam_node.py` 直接读取原始 MJPG，利用 libjpeg 的 DCT 缩放以 1/4 尺寸解码（约 25 ms），再缩放到 640×480 发布。

```bash
ros2 launch vln_robot_bringup robot.launch.py                          # 默认 camera_driver:=wide
ros2 launch vln_robot_bringup robot.launch.py camera_driver:=usb_cam   # 回退到裁切的 640x480
```

话题保持不变：`/camera/image_raw`（bgr8）和 `/camera/camera_info`。参数在 `config/wide_cam.yaml`，内参读取 `config/camera_info.yaml`。usb_cam 模式使用 `config/usb_cam.yaml` 和 `config/camera_info_640crop.yaml`。
关闭自动曝光/对焦（可选）：

```bash
v4l2-ctl -d /dev/vln_cam -c focus_automatic_continuous=0 -c focus_absolute=0
```

## 3. 标定（必做）

内参会同时用于（1）服务器端 `--intrinsic` 和（2）雷达投影深度。

推荐使用 `tools/calibrate_camera.py`。它不依赖 ROS，可以通过 SSH 远程运行，自动采图并提示还缺哪些姿态：

```bash
pkill -f "wide_cam_node|usb_cam"   # 先停掉占用相机的进程
# 用系统自带的 python3，不要先 source ros_env.sh
python3 ~/robot-vln/tools/calibrate_camera.py --capture 4000x3000 --cols 8 --rows 6   # 全传感器，对应 wide_cam_node
```

- `--cols` 和 `--rows` 填**内角点**数量：10×7 个格子的棋盘是 9×6 个内角点。`--square` 填格子边长，单位米。
- `--capture 4000x3000` 按 wide_cam_node 完全相同的方式取图（先全幅采集，再缩放到 `--width/--height`，默认 640×480）。不加该参数时，会直接用 640×480 裁切模式，对应 usb_cam，此时请加 `--out .../camera_info_640crop.yaml`。广角下畸变更大，四个角一定要采到。
- 实时画面会显示在 M11 屏幕上，也可以在浏览器打开 `http://<机器人IP>:8090/` 查看。
- 采够 25 张且覆盖到各个位置、远近和倾斜角度后，脚本会自动计算，结果直接写入 `config/camera_info.yaml`，并打印服务器启动用的 `--intrinsic` 参数。
- 原始采集图片保存在 `~/calib_<时间>/`，其中 `check_raw_vs_undistorted.jpg` 用来检查去畸变效果。

也可以用 ROS 自带的 GUI 工具（需要显示器）：`ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025 --ros-args -r image:=/camera/image_raw -r camera:=/camera`。

## 4. 外参

在 `config/extrinsics.yaml` 中填写 `camera_link` 相对 `base_link` 的位置（x 前、y 左、z 上，单位 m）。
如果要做雷达到相机的精确标定，可以参考 livox_camera_calib / direct_visual_lidar_calibration。

### 相机标定结果（2026-11-02）

全传感器广角（默认，`camera_info.yaml`）：
- 采集 4000×3000，输出 640×480：fx 493.4、fy 494.0、cx 325.3、cy 241.0；重投影误差 RMS 0.098 px，共 36 张图。
- 视场角：水平 65.9°，垂直 51.8°。畸变 k1 +0.131、k2 -0.416、k3 +0.361。
- GPU 服务器参数：`--intrinsic 493.4 494.0 325.3 241.0`。

640×480 裁切（`camera_info_640crop.yaml`，usb_cam 模式）：
- fx 689.7、fy 691.1、cx 324.4、cy 250.0；RMS 0.12 px，共 33 张图；水平约 50°，垂直约 38°。