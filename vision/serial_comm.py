"""Serial transport for the FPGA vision UART protocol."""
from __future__ import annotations

import serial

from serial_frames import build_frame


class SerialComm:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self._port = serial.Serial(port=port, baudrate=baudrate, timeout=0)

    def send_six_data(self, *data: object) -> None:
        if len(data) != 6:
            raise ValueError("the FPGA object frame requires six values")
        self._port.write(build_frame(*data))

    def close(self) -> None:
        self._port.close()
