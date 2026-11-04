# Vision tools

The vision programs run on the host computer or an attached Linux SBC and send detection results to the FPGA through the vision UART.

- `object_detector.py` runs the ONNX detector and sends two tracked object centers.
- `qr_scanner.py` detects QR codes `0` through `4` and sends the QR center and route ID.
- `serial_frames.py` defines the binary frame format.
- `serial_comm.py` owns the pyserial transport.

Use the same 115200 baud rate and connect the host transmitter to the FPGA `vision_rx` pin. Coordinates are based on a 640×480 camera frame.
