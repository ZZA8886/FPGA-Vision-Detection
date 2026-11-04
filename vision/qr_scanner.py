"""Camera QR scanner with optional JSON output for the serial bridge."""
from __future__ import annotations
import argparse
import json
import cv2
from serial_frames import build_qr_frame

VALID_CODES = frozenset("01234")


def qr_scan(frame, detector=None):
    detector = detector or cv2.QRCodeDetector()
    text, points, _ = detector.detectAndDecode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if not text or points is None:
        return None
    polygon = points[0].astype(int)
    cv2.polylines(frame, [polygon], True, (0, 255, 0), 2)
    cv2.putText(frame, text, tuple(polygon[0] + (0, 50)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
    if text not in VALID_CODES:
        return None
    center = polygon.mean(axis=0).round().astype(int)
    return int(center[0]), int(center[1]), int(text)


def run(camera=0, port="COM3", baudrate=115200, stdout_serial=False):
    serial_port = None
    if not stdout_serial:
        import serial
        serial_port = serial.Serial(port=port, baudrate=baudrate)
    capture = cv2.VideoCapture(camera)
    detector = cv2.QRCodeDetector()
    try:
        while capture.isOpened():
            ok, frame = capture.read()
            if not ok:
                print("Camera read failed")
                break
            result = qr_scan(frame, detector)
            if result is not None:
                if stdout_serial:
                    print(json.dumps({"qr_data": result}), flush=True)
                else:
                    serial_port.write(build_qr_frame(*result))
                print(f"QR detected: {result}")
            cv2.imshow("QR scanner", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()
        if serial_port is not None:
            serial_port.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--stdout-serial", action="store_true")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()
    run(camera=args.camera, port=args.port, baudrate=args.baudrate,
        stdout_serial=args.stdout_serial)
