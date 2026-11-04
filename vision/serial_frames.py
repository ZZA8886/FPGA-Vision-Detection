"""Binary frame encoding shared by the camera applications and the bridge."""
from __future__ import annotations
import math
from collections.abc import Iterable

FRAME_START = 0xAA
FRAME_END = 0x55


def _uint16(value: object) -> int:
    try:
        number = float(value) if value is not None else 0.0
    except (TypeError, ValueError):
        number = 0.0
    if not math.isfinite(number):
        number = 0.0
    return max(0, min(65535, int(round(number))))


def build_frame(*values: object) -> bytes:
    payload = bytearray((FRAME_START,))
    for value in values:
        payload.extend(_uint16(value).to_bytes(2, "big"))
    payload.append(FRAME_END)
    return bytes(payload)


def build_qr_frame(x: object, y: object, class_id: object) -> bytes:
    if isinstance(class_id, bool) or not isinstance(class_id, int) or not 0 <= class_id <= 4:
        raise ValueError("QR class must be an integer from 0 to 4")
    x_value, y_value = _uint16(x), _uint16(y)
    return bytes((FRAME_START, x_value >> 8, x_value & 255,
                  y_value >> 8, y_value & 255, class_id, FRAME_END))


def frame_from_iterable(values: Iterable[object]) -> bytes:
    return build_frame(*tuple(values))
