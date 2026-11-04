import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


CLASSES = ["0", "1"]  # 类别映射


class StdoutSerialComm:
    def send_six_data(self, *data):
        if len(data) != 6:
            raise ValueError("requires exactly 6 values")
        print(json.dumps({"serial_data": data}), flush=True)


class ObjectDetector:
    def __init__(self, model_path, comm=None, port="COM3", baudrate=115200):
        self.session = ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
        self.input_size = (320, 320)
        self.fps_time = time.time()
        self.fps = 0.0
        if comm is None:
            from serial_comm import SerialComm
            comm = SerialComm(port, baudrate)
        self.comm = comm

    def detect(self, img):
        # 预处理（合并缩放、填充、归一化）
        h, w = img.shape[:2]
        scale = min(self.input_size[0] / w, self.input_size[1] / h)
        new_w, new_h = int(w * scale), int(h * scale)
        pad_x, pad_y = (self.input_size[0] - new_w) // 2, (self.input_size[1] - new_h) // 2

        # 缩放+填充
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (new_w, new_h))
        img_input = np.zeros((self.input_size[1], self.input_size[0], 3), np.uint8)
        img_input[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = img_resized

        # 格式转换
        img_input = img_input.transpose((2, 0, 1))[np.newaxis, ...].astype(np.float32) / 255.0
        input_name = self.session.get_inputs()[0].name
        outputs = self.session.run(None, {input_name: img_input})

        targets = {}
        if outputs and len(outputs[0]) > 0:
            # ========== 兼容多维数组解析 ==========
            dets = outputs[0].reshape(-1, outputs[0].shape[-1])  # 展平为[N,6]
            for det in dets:
                if len(det) < 6:
                    continue
                # 适配不同YOLO输出顺序
                if 0 <= det[4] <= 1:
                    x1, y1, x2, y2, prob, lab = det[0], det[1], det[2], det[3], det[4], det[5]
                else:
                    prob, x1, y1, x2, y2, lab = det[0], det[1], det[2], det[3], det[4], det[5]
                if prob > 0.6:
                    # 确保类别ID是标量，避免数组转int报错
                    lab = int(np.squeeze(lab))
                    if lab not in (0, 1):
                        continue
                    # 坐标还原（核心解决框偏下）
                    x1, y1 = (x1 - pad_x) / scale, (y1 - pad_y) / scale
                    x2, y2 = (x2 - pad_x) / scale, (y2 - pad_y) / scale
                    center_x, center_y = (int)((x1 + x2) / 2), (int)((y1 + y2) / 2)

                    if lab not in targets or prob > targets[lab][0]:
                        targets[lab] = (prob, center_x, center_y)

                    # 绘制框和文字
                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    text_y = max(int(y1) - 10, 20)
                    cv2.putText(img, CLASSES[lab], (int(x1), text_y),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
        if targets:
            x0, y0 = targets[0][1:] if 0 in targets else (0, 0)
            x1, y1 = targets[1][1:] if 1 in targets else (0, 0)
            data = (x0, y0, 0, x1, y1, 1)
            self.comm.send_six_data(*data)
            print(f"Detected: {data}")
        curr_time = time.time()
        self.fps = 1.0 / (curr_time - self.fps_time)
        self.fps_time = curr_time
        cv2.putText(img, f"FPS: {self.fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return img


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run ONNX object tracking for navbot.")
    parser.add_argument("--model", type=Path, default=Path(__file__).parent / "models" / "e.onnx")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--stdout-serial", action="store_true")
    args = parser.parse_args()

    if args.stdout_serial:
        comm = StdoutSerialComm()
    else:
        from serial_comm import SerialComm
        comm = SerialComm(args.port, args.baudrate)
    detector = ObjectDetector(args.model, comm=comm)
    cap = cv2.VideoCapture(args.camera)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 主循环
    while True:
        ret, frame = cap.read()
        if not ret:
            print("摄像头读取失败")
            break

        frame = detector.detect(frame)
        cv2.imshow('Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
