import sys
import argparse
import os
from pathlib import Path

import torch
import numpy as np
from ultralytics import YOLO
import cv2

class Result:
    def __init__(self, xyxy=(0.0, 0.0, 0.0, 0.0), name='', conf=0.0, track_id=None):
        self.u1 = float(xyxy[0])
        self.v1 = float(xyxy[1])
        self.u2 = float(xyxy[2])
        self.v2 = float(xyxy[3])
        self.name = name
        self.conf = float(conf)
        self.track_id = track_id
    
    def __repr__(self):
        track_str = f", ID={self.track_id}" if self.track_id is not None else ""
        return (f"Result(bbox=[{self.u1:.1f}, {self.v1:.1f}, {self.u2:.1f}, {self.v2:.1f}], "
                f"name='{self.name}', conf={self.conf:.3f}{track_str})")


class Detector:
    def __init__(
        self,
        weights="yolo11n.pt",
        imgsz=(640, 640),
        conf_thres=0.25,
        iou_thres=0.45,
        device='',
        view_img=True,
        line_thickness=3,
        hide_labels=False,
        hide_conf=False,
        half=False,
        classes=None
    ):
        device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = YOLO(weights)
        self.model.to(device)

        self.imgsz = imgsz
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.device = device
        self.view_img = view_img
        self.line_thickness = line_thickness
        self.hide_labels = hide_labels
        self.hide_conf = hide_conf
        self.half = half
        
        # FIX: Handle classes parameter - convert to list if needed
        if classes is not None:
            if isinstance(classes, int):
                self.classes = [classes]
            elif isinstance(classes, (list, tuple)):
                self.classes = list(classes)
            else:
                self.classes = None
        else:
            self.classes = None
    
    @torch.no_grad()
    def detect(self, img0):
        try:
            img_detect = cv2.rotate(img0, cv2.ROTATE_90_COUNTERCLOCKWISE)

            results = self.model.track(
                img_detect,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                iou=self.iou_thres,
                classes=self.classes,
                device=self.device,
                half=self.half,
                verbose=False,
                persist=True  # Keep track IDs persistent across frames
            )

            r = results[0]
            detections = []
            img_display = img_detect.copy()

            # Check if there are any detections
            if r.boxes is None or len(r.boxes) == 0:
                return img_display, detections

            # Get all detection data
            boxes = r.boxes.xyxy.cpu().numpy()
            classes_arr = r.boxes.cls.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            
            # Get tracking IDs if available
            if r.boxes.id is not None:
                track_ids = r.boxes.id.cpu().numpy()
            else:
                track_ids = [None] * len(boxes)

            # Process each detection
            for xyxy, cls, conf, track_id in zip(boxes, classes_arr, confs, track_ids):
                cls = int(cls)
                name = self.model.names[cls]
                tid = int(track_id) if track_id is not None else None

                detections.append(Result(xyxy, name, conf, tid))

                # Draw bounding boxes
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                
                # Use different color for each person ID
                color = self._get_color(tid) if tid is not None else (0, 255, 0)
                
                # Draw rectangle
                cv2.rectangle(img_display, (x1, y1), (x2, y2), color, self.line_thickness)
                
                # Prepare label with ID
                label = "" if self.hide_labels else name
                if tid is not None:
                    label += f" ID:{tid}" if label else f"ID:{tid}"
                if not self.hide_conf:
                    label += f" {conf:.2f}" if label else f"{conf:.2f}"
                
                # Draw label background and text
                if label:
                    (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(img_display, (x1, y1 - label_h - 10), (x1 + label_w, y1), color, -1)
                    cv2.putText(img_display, label, (x1, y1 - 5),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            return img_display, detections
            
        except Exception as e:
            print(f"Detection error: {e}")
            import traceback
            traceback.print_exc()
            return img0, []
    
    def _get_color(self, track_id):
        """Generate consistent color for each track ID"""
        if track_id is None:
            return (0, 255, 0)
        # Use track_id as seed for consistent colors
        np.random.seed(int(track_id) * 50)
        return tuple(np.random.randint(50, 255, 3).tolist())


def parse_opt(args=None):
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='YOLO Person Detection with ROS2')
    parser.add_argument("--weights", type=str, default="yolo11n.pt", help="Model weights path")
    parser.add_argument("--imgsz", nargs="+", type=int, default=[640], help="Inference size")
    parser.add_argument("--conf-thres", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IOU threshold")
    parser.add_argument("--view-img", action="store_true", help="Show results")
    parser.add_argument("--classes", nargs="+", type=int, default=[0], help="Filter by class (0=person)")
    parser.add_argument("--line-thickness", type=int, default=3, help="Bounding box thickness")
    parser.add_argument("--hide-labels", action="store_true", help="Hide labels")
    parser.add_argument("--hide-conf", action="store_true", help="Hide confidence")
    parser.add_argument("--half", action="store_true", help="Use FP16 half-precision")
    parser.add_argument("--device", type=str, default="", help="Device (cuda/cpu)")
    
    # Parse only known args to ignore ROS2 arguments
    opt, unknown = parser.parse_known_args(args)

    # Handle image size
    if len(opt.imgsz) == 1:
        opt.imgsz = opt.imgsz * 2
    
    return opt
