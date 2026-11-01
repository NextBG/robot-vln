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

```bash
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

标定完成后点击 SAVE，把 `/tmp/calibrationdata.tar.gz` 里的 `ost.yaml` 内容填进 `config/camera_info.yaml`。

## 4. 外参

在 `config/extrinsics.yaml` 中填写 `camera_link` 相对 `base_link` 的位置（x 前、y 左、z 上，单位 m）。
如果要做雷达到相机的精确标定，可以参考 livox_camera_calib / direct_visual_lidar_calibration。
