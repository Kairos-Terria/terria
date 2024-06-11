import torch
import torchvision

from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords

class YOLOv7:
    def __init__(self):
        self.weights = '/home/terria/catkin_ws/src/terria_ros/scripts/model_utils/yolov7/yolov7-tiny.pt'
        self.device = 'cuda'
        self.model = attempt_load(self.weights, map_location=self.device)
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        
    def get_pred(self, img):

        def frame_to_tensor(img):
            img = torch.from_numpy(img).to(self.device)
            img = img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            img = img.permute(2, 0, 1).unsqueeze(0)
            img = torchvision.transforms.functional.resize(img, (640, 640))

            return img

        img = frame_to_tensor(img)

        with torch.no_grad():
            pred = self.model(img, augment=False)[0]

        pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=False)

        boxes, labels = list(), list()
        for det in pred:
            if len(det):
                #det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img.shape).round()
                det[:, :4] = scale_coords([720, 1280], det[:, :4], [1, 3, 720, 1280]).round()

                for *xyxy, conf, cls in reversed(det):
                    xyxy = [int(e.item()) for e in xyxy]
                    label = f'{self.names[int(cls)]}'

                    boxes.append(xyxy)
                    labels.append(label)
                    
        return boxes, labels 

"""
from torchvision.models.detection import *

class SSDLite:
    def __init__(self):
        self.model = ssdlite320_mobilenet_v3_large(weights=SSDLite320_MobileNet_V3_Large_Weights.DEFAULT)

        self.model.to('cpu')
        self.model.eval()

        weights = SSDLite320_MobileNet_V3_Large_Weights.verify(SSDLite320_MobileNet_V3_Large_Weights.DEFAULT)
        self.class_name = weights.meta["categories"]

    def get_pred(self, img):
        def frame_to_tensor(img):
            img = torch.from_numpy(img).to('cpu')
            img = img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            img = img.permute(2, 0, 1).unsqueeze(0)

            return img

        img = frame_to_tensor(img)

        with torch.no_grad():
            pred = self.model(img)[0]

            inds = pred['scores'] > 0.5
            boxes = pred['boxes'][inds]
            scores = pred['scores'][inds]
            labels = pred['labels'][inds]

            boxes = [[int(e) for e in b] for b in boxes]
            labels = [self.class_name[l] for l in labels]

            return boxes, labels
"""
