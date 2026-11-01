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

## 2. ROS 2 驱动：usb_cam

```bash
sudo apt install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ros2_ws/src/vln_robot_bringup/config/usb_cam.yaml
```

`config/usb_cam.yaml` 中 `pixel_format: mjpeg2rgb`，分辨率 640×480，帧率 15 fps。话题为 `/camera/image_raw` 和 `/camera/camera_info`。

关闭自动曝光/对焦（可选）：

```bash
v4l2-ctl -d /dev/vln_cam -c focus_automatic_continuous=0 -c focus_absolute=0
```

## 3. 标定（必做）

内参会同时用于（1）服务器端 `--intrinsic` 和（2）雷达投影深度。

推荐使用 `tools/calibrate_camera.py`。它不依赖 ROS，可以通过 SSH 远程运行，自动采图并提示还缺哪些姿态：

```bash
pkill -f usb_cam   # 先停掉占用相机的进程
# 用系统自带的 python3，不要先 source ros_env.sh
python3 ~/robot-vln/tools/calibrate_camera.py --cols 9 --rows 6 --square 0.025
```

- `--cols` 和 `--rows` 填**内角点**数量：10×7 个格子的棋盘是 9×6 个内角点。`--square` 填格子边长，单位米。
- 分辨率默认 640×480，必须和 `usb_cam.yaml` 一致，换分辨率要重新标定。
- 实时画面会显示在 M11 屏幕上，也可以在浏览器打开 `http://<机器人IP>:8090/` 查看。
- 采够 25 张且覆盖到各个位置、远近和倾斜角度后，脚本会自动计算，结果直接写入 `config/camera_info.yaml`，并打印服务器启动用的 `--intrinsic` 参数。
- 原始采集图片保存在 `~/calib_<时间>/`，其中 `check_raw_vs_undistorted.jpg` 用来检查去畸变效果。

也可以用 ROS 自带的 GUI 工具（需要显示器）：`ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.025 --ros-args -r image:=/camera/image_raw -r camera:=/camera`。

## 4. 外参

在 `config/extrinsics.yaml` 中填写 `camera_link` 相对 `base_link` 的位置（x 前、y 左、z 上，单位 m）。
如果要做雷达到相机的精确标定，可以参考 livox_camera_calib / direct_visual_lidar_calibration。

### 相机标定结果（2026-11-02）

- 640×480 下 fx 689.7、fy 691.1、cx 324.4、cy 250.0；重投影误差 RMS 0.12 px，共 33 张图。
- 视场角：水平约 50°，垂直约 38°。640×480 是从 16:9 传感器中间裁出来的，所以视场比较窄。
- 畸变小：画面角落的偏移约 2%。
- GPU 服务器已改用 `--intrinsic 689.7 691.1 324.4 250.0` 重启。
- 注意：InternVLA-N1 训练时用的相机视场更宽。如果实测效果不好，可以试试 1280×720 模式，需要重新标定。