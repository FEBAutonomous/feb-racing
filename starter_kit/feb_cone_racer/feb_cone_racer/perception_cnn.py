"""Cone perception with a trained detector instead of colour thresholds: the same lidar clustering,
the same bearing matching and the same colour carry-over as perception.Perception, but the camera
blobs come from a YOLO model (ONNX, run with onnxruntime) whose class is the colour. No venue
tuning: lighting is the detector's problem. A box with no lidar cone on its bearing is never used,
so a false detection on a jacket or a bag cannot create a cone; only a lidar cluster can.

Classes as trained in ~/FEB/cone-detector: 0 blue, 1 yellow, 2 orange.
"""
import math

import cv2
import numpy as np

from .perception import BLUE, ORANGE, YELLOW, Perception

CLASS_COLOUR = {0: BLUE, 1: YELLOW, 2: ORANGE}


class CnnPerception(Perception):
    def __init__(self, p, model_path, conf=0.3, iou=0.5, size=640):
        super().__init__(p)
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = max(1, min(4, __import__("os").cpu_count() or 1))
        providers = [pr for pr in ("CUDAExecutionProvider", "TensorrtExecutionProvider", "CPUExecutionProvider") if pr in ort.get_available_providers()]
        self.session = ort.InferenceSession(model_path, opts, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        self.conf, self.iou, self.size = conf, iou, size
        self.detections = []            # [(class, conf, x0, y0, x1, y1)] of the latest image, in image pixels
        self.provider = self.session.get_providers()[0]

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            img = img[:, :, ::-1]                    # the model was trained on RGB
        self.image_width = msg.width
        h, w = img.shape[:2]
        # letterbox to size x size
        s = self.size / max(h, w)
        nh, nw = int(round(h * s)), int(round(w * s))
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        top, left = (self.size - nh) // 2, (self.size - nw) // 2
        canvas[top:top + nh, left:left + nw] = cv2.resize(np.ascontiguousarray(img), (nw, nh), interpolation=cv2.INTER_LINEAR)
        x = canvas.astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        out = self.session.run(None, {self.input_name: x})[0]
        pred = out[0].T if out.shape[1] < out.shape[2] else out[0]      # (N, 4 + classes)
        boxes, scores, classes = pred[:, :4], pred[:, 4:].max(1), pred[:, 4:].argmax(1)
        keep = scores >= self.conf
        boxes, scores, classes = boxes[keep], scores[keep], classes[keep]
        dets = []
        if len(boxes):
            xyxy = np.c_[boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2, boxes[:, 0] + boxes[:, 2] / 2, boxes[:, 1] + boxes[:, 3] / 2]
            idx = cv2.dnn.NMSBoxes([[float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])] for b in xyxy], scores.astype(float).tolist(), self.conf, self.iou)
            for i in np.array(idx).reshape(-1):
                b = (xyxy[i] - [left, top, left, top]) / s
                x0, y0, x1, y1 = [float(np.clip(v, 0, w if k % 2 == 0 else h)) for k, v in enumerate(b)]
                dets.append((int(classes[i]), float(scores[i]), x0, y0, x1, y1))
        self.detections = dets
        # the blob list Perception.assign expects: colour, column, top row, bottom row, area, dim.
        # dim stays False (the detector already judged it is a cone); confidence rides along as a 7th item
        self.blobs = [(CLASS_COLOUR.get(c, BLUE), (x0 + x1) / 2.0, y0, y1, (x1 - x0) * (y1 - y0), False, conf) for c, conf, x0, y0, x1, y1 in dets]
