from ultralytics import YOLO
import cv2
import math
import json

class VisionSystem:
    """
    YOLOで人を検出/追跡しつつ、画像内のArUco(ID)を読み取り、
    人ボックス内にあるArUcoを優先IDとして採用する。
    """
    def __init__(self, weights, conf_thres, imgsz,
                 aruco_dict_type=cv2.aruco.DICT_4X4_50,
                 aruco_mapping_json="aruco_mapping.json",
                 yolo_id_offset=10000):
        self.model = YOLO(weights, task='detect')
        self.conf_thres = conf_thres
        self.imgsz = imgsz
        self.camera_info = None

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # YOLO track_id -> aruco_id の対応を保持（マーカーが一瞬見えなくてもArUco優先にする）
        self.track_to_aruco = {}

        # aruco_mapping.json（任意）
        self.aruco_mapping = {}
        try:
            with open(aruco_mapping_json, "r", encoding="utf-8") as f:
                self.aruco_mapping = json.load(f)
        except FileNotFoundError:
            pass

        # YOLO fallback ID をMarkerでも扱えるようにintにするためのオフセット
        self.yolo_id_offset = int(yolo_id_offset)

    def set_camera_info(self, msg):
        self.camera_info = msg

    def _detect_aruco(self, bgr_image):
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)

        markers = {}
        if ids is None:
            return markers, corners, ids

        for i, marker_id in enumerate(ids.flatten()):
            pts = corners[i][0]  # shape (4,2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            markers[int(marker_id)] = (cx, cy, pts)

        return markers, corners, ids

    @staticmethod
    def _point_in_box(px, py, x1, y1, x2, y2):
        return (x1 <= px <= x2) and (y1 <= py <= y2)

    def _person_key_from_track(self, track_id: int) -> int:
        """ArUco紐付けがあればArUco ID、無ければ(オフセット+track_id)を返す"""
        if track_id in self.track_to_aruco:
            return int(self.track_to_aruco[track_id])
        return self.yolo_id_offset + int(track_id)

    def detect_people_angles(self, cv_image):
        """
        return:
          people_dict: {person_id(int): theta(float)}
          annotated_frame: デバッグ用の描画画像
        """
        # 1) ArUco検出
        markers, ar_corners, ar_ids = self._detect_aruco(cv_image)

        # 2) YOLO tracking
        results = self.model.track(
            cv_image,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            persist=True,
            verbose=False
        )

        annotated_frame = cv_image.copy()
        if results and len(results) > 0:
            annotated_frame = results[0].plot()

        # ArUcoも描画
        if ar_ids is not None and len(ar_ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated_frame, ar_corners, ar_ids)

        if self.camera_info is None:
            return {}, annotated_frame

        # ★カメラ内部パラメータ（回転させてない前提）
        # PeopleMapperNode側で画像を回転しているなら、ここも合わせる必要があります（後述）
        fx = self.camera_info.k[0]
        cx = self.camera_info.k[2]

        # 3) 人ボックス一覧
        yolo_people = []  # (track_id, x1,y1,x2,y2)
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id != 0:  # person
                    continue
                if box.id is None:
                    continue

                track_id = int(box.id[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                yolo_people.append((track_id, float(x1), float(y1), float(x2), float(y2)))

        # 4) ArUco中心が入っている人ボックスへ紐付け（track_id -> aruco_id）
        for aruco_id, (mx, my, _pts) in markers.items():
            matched_track = None
            best_area = None
            for (track_id, x1, y1, x2, y2) in yolo_people:
                if self._point_in_box(mx, my, x1, y1, x2, y2):
                    area = (x2 - x1) * (y2 - y1)
                    if best_area is None or area < best_area:
                        best_area = area
                        matched_track = track_id
            if matched_track is not None:
                self.track_to_aruco[matched_track] = aruco_id

        # 5) 出力（ArUco優先ID）
        people_dict = {}
        for (track_id, x1, y1, x2, y2) in yolo_people:
            u_center = (x1 + x2) / 2.0
            theta = math.atan2((u_center - cx), fx)

            person_id = self._person_key_from_track(track_id)
            people_dict[person_id] = theta

        return people_dict, annotated_frame