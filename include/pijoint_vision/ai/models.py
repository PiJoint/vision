import random
import enum
from pathlib import Path

import torch
from yolov7.models.experimental import attempt_load
from yolov7.utils.datasets import LoadImages
from yolov7.utils.plots import plot_one_box
from yolov7.utils.torch_utils import select_device,  TracedModel
from yolov7.utils.general import check_img_size,   non_max_suppression,  \
    scale_coords

from .images import LoadImage

class Model:


    def __init__(self, w, tresh, device='', augment=False) -> None:
        self.augment = augment
        self.tresh = tresh
        self.device = select_device(device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        self.model = attempt_load(w, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(640, s=self.stride)  # check img_size

        if self.half:
            self.model.half()  # to FP16

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        if self.half:
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(device).type_as(next(self.model.parameters())))  # run once



    def detect_object(self, source):
        ob = []

        dataset = LoadImage(source, img_size=self.imgsz, stride=self.stride)
        old_img_w = old_img_h = self.imgsz
        old_img_b = 1

        for path, img, im0s, _ in dataset:
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            
            if self.half and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
                old_img_b = img.shape[0]
                old_img_h = img.shape[2]
                old_img_w = img.shape[3]
                for i in range(3):
                    self.model(img, augment=self.augment)[0]

            with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
                pred = self.model(img, augment=self.augment)[0]
            
            pred = non_max_suppression(pred, self.tresh, 0.45, classes=None, agnostic=True)
            for i, det in enumerate(pred):  # detections per image
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

                p = Path(p)  # to Path
                
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    for *xyxy, conf, cls in reversed(det):
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        x, y,x1,y1 = xyxy
                        ob.append(
                            (int(cls), (int(x), int(y), int(x1), int(y1)), conf)
                        )
            break
        return ob
        
    def plot(self, img0, xyxy, cls):
        plot_one_box(xyxy, img0, label=self.names[cls], color=self.colors[cls], line_thickness=1)