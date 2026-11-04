"""Queued serial output used by the object detector."""
import queue
import threading
import serial
from serial_frames import build_frame


class SerialComm:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0, write_timeout=0)
        self._queue = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

    def _write_loop(self):
        while not self._stop.is_set():
            try:
                packet = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.ser.write(packet)
            except (serial.SerialException, serial.SerialTimeoutException):
                pass
            finally:
                self._queue.task_done()

    def send_six_data(self, *values):
        if len(values) != 6:
            raise ValueError("requires exactly 6 values")
        packet = build_frame(*values)
        try:
            self._queue.put_nowait(packet)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(packet)
            except queue.Full:
                pass

    def close(self):
        self._stop.set()
        self._writer.join(timeout=0.5)
        self.ser.close()
