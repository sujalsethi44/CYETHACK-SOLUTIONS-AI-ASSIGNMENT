import cv2
import numpy as np
from ultralytics import YOLO

class YOLODetector:
    def __init__(self, model_path='yolov8n.pt'):
        self.model = YOLO(model_path)
        self.target_classes = ['car', 'person']
        self.class_mapping = {'car': 'Car', 'person': 'Pedestrian'}
        
    def detect(self, image):
        results = self.model(image, verbose=False)
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id].lower()
                    
                    if class_name in self.target_classes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf[0])
                        
                        if confidence > 0.5:
                            detections.append({
                                'bbox': [x1, y1, x2, y2],
                                'confidence': confidence,
                                'class': self.class_mapping[class_name]
                            })
        
        return detections
