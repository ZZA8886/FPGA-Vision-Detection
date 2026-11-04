# FPGA-Vision-Detection
基于 Python、OpenCV 和 ONNX Runtime 的 FPGA 视觉识别项目，支持摄像头二维码识别与目标检测。系统可获取目标位置、类别等信息，并通过统一的数据帧格式输出识别结果，适用于嵌入式视觉控制、智能小车和 FPGA 综合实践等场景。

## 安装与运行

需要 Python 3.10 或更新版本、USB 摄像头。使用串口输出时需要连接串口设备，默认端口为 Linux 的 `/dev/ttyUSB0`，波特率为 115200；Windows 用户请在代码中改为实际 COM 端口。

```bash
pip install -r requirements.txt
python scareQR.py
python onnx_infer.py
```

也可以用 JSON 标准输出代替串口输出：

```bash
python scareQR.py --stdout-serial
python onnx_infer.py --stdout-serial
```

按 `q` 退出摄像头窗口。目标检测默认加载项目内的 `e.onnx`，`s.onnx` 为备用模型。

## 文件说明

- `scareQR.py`：识别内容为 0–4 的二维码，输出中心坐标及类别。
- `onnx_infer.py`：ONNX 目标检测，输出类别 0、1 的中心坐标。
- `serial_frames.py`：编码视觉结果的数据帧。
- `serial_transport.py`：目标检测使用的异步串口输出依赖。
- `e.onnx`、`s.onnx`：模型文件。

此仓库包含视觉端 Python 程序，不包含 FPGA 硬件描述代码。
