# navbot-robot-system

FPGA motion control and camera vision software for the Navbot platform.

## Repository layout

```text
fpga/     Gowin project, Verilog RTL, camera pipeline and board constraints
vision/   Python object detection, QR scanning and FPGA serial transport
```

The FPGA project is `fpga/navbot_camera.gprj`. Its `navbot_camera_top` integrates the mecanum controller, OV5647 MIPI camera path, HDMI output, vision UART, QR route runner and IMU660RA yaw feedback.

## Vision protocol

The FPGA vision UART uses 115200 8N1. Object mode accepts 14 bytes:

```text
AA x0[15:8] x0[7:0] y0[15:8] y0[7:0] 00 00
   x1[15:8] x1[7:0] y1[15:8] y1[7:0] 00 01 55
```

QR mode accepts 7 bytes:

```text
AA x[15:8] x[7:0] y[15:8] y[7:0] class_id 55
```

The Python encoders in `vision/serial_frames.py` match these frames.

## Running the vision side

```powershell
cd vision
python -m pip install -r requirements.txt
python object_detector.py --model models/e.onnx --port COM3
python qr_scanner.py --port COM3
```

Use `--stdout-serial` to inspect generated frames without opening a serial port. The ONNX files in `vision/models/` are runtime assets; replace them only when the input and output layout stays compatible with `object_detector.py`.

## FPGA workflow

Open `fpga/navbot_camera.gprj` in Gowin IDE, select `navbot_camera_top` as the top module, then synthesize and generate the bitstream. The board constraints are in `fpga/constraints/navbot_camera.cst` and `fpga/constraints/navbot_camera.sdc`.
