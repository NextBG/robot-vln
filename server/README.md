# server/

在 GPU 机器上运行的 VLN 推理服务。详见 [docs/setup/gpu-server.md](../docs/setup/gpu-server.md)。

- `setup_server.sh`：安装 InternNav 环境并下载 InternVLA-N1-DualVLN 权重
- `test_client.py`：不用机器人，发一帧图像测试服务（`python server/test_client.py --image xx.jpg`）
- `internvla_n1_server.py`：基于官方 `http_internvla_server.py` 的 HTTP 服务（端口 5801，`POST /eval_dual`，`GET /health`）
