from ultralytics import YOLO
import math

class VisionSystem:
    def __init__(self, weights, conf_thres, imgsz):
        self.model = YOLO(weights, task='detect')
        self.conf_thres = conf_thres
        self.imgsz = imgsz
        self.camera_info = None

    def set_camera_info(self, msg):
        self.camera_info = msg

    def detect_people_angles(self, cv_image):
        """
        人間が検出された位置の角度 [rads] リストを返す
        
        :param cv_image: フレームごとの画像
        """
        
        # run YOLO
        # results = self.model(cv_image, imgsz=self.imgsz, verbose=False, stream=True)
        results = self.model.track(cv_image, imgsz=self.imgsz, conf=self.conf_thres, persist=True, verbose=False)
        annotated_frame = results[0].plot()

        if self.camera_info is None:
            return [], annotated_frame

        fx = self.camera_info.k[4]
        cx = self.camera_info.k[5]
        people_data = {}

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])

                if cls_id == 0:
                    if box.id is not None:
                        track_id = int(box.id[0])
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        u_center = (x1+x2)/2.0

                        # calculate angle relative to camera center
                        theta = math.atan2(u_center-cx, fx)
                        people_data[track_id] = theta

        return people_data, annotated_frame