import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import cv2

class LidarSystem:
    def __init__(self):
        self.latest_scan = None
        self.scan_ranges = None
        self.angle_min = 0
        self.angle_inc = 0

    def update_scan(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.latest_scan = msg

    def get_distance_at_angle(self, angle_rad, offset_rad=0.0):
        """
        Calculates the closest valid distance within a small cone around the target angle.
        """
        if self.latest_scan is None:
            return None
        
        # 1. Calculate Target Angle
        raw_angle = angle_rad + offset_rad
        target_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        if angle_inc == 0:
            return None

        # 2. Convert Angle to Index
        center_idx = int(round((target_angle - angle_min) / angle_inc))
        num_ranges = len(scan.ranges)

        # 3. Define a "Search Cone" (e.g., +/- 3 degrees)
        # 3 degrees ~= 0.05 radians
        search_radius_rad = 0.02
        search_radius_idx = int(search_radius_rad / angle_inc)
        
        valid_distances = []

        # 4. Loop through indices in the cone
        for i in range(center_idx - search_radius_idx, center_idx + search_radius_idx + 1):
            # Handle wrap-around (e.g., -5 becomes 355)
            # This is crucial for 360 Lidar
            idx = i % num_ranges 
            
            dist = scan.ranges[idx]
            
            # Filter bad data (inf, nan, too close)
            # Turtlebot Lidar often returns 0.0 for errors
            if dist > 0.05 and not math.isinf(dist) and not math.isnan(dist):
                valid_distances.append(dist)

        # 5. Return the closest distance found in the cone
        if valid_distances:
            # We assume the closest point in the cone is the person
            min_dist = min(valid_distances)
            return min_dist, target_angle
            
        return None

    """
    def get_min_distance_to_sur(self, person_angle, person_dist, offset_rad=0.0):
        
        # 人から最も近い壁からの距離を検出 (60度セクター)
        
        if self.latest_scan is None:
            return None
        
        raw_person_angle = person_angle + offset_rad
        scan = self.latest_scan

        # LiDARフレーム上、人の位置の角度
        person_theta = math.atan2(math.sin(raw_person_angle), math.cos(raw_person_angle))
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        center_idx = int(round(person_theta - angle_min) / angle_inc)

        # 検出エリアを定義 (60度左右)
        search_radius_rad = 1.0
        search_width_idx = int(search_radius_rad / angle_inc)

        num_ranges = len(scan.ranges)
        min_wall_dist = 999.0
        found_wall = False

        for i in range(center_idx - search_width_idx, center_idx + search_width_idx + 1):
            idx = i % num_ranges

            # ロボットから壁の距離
            r_scan = scan.ranges[idx]

            if r_scan < 0.1 or math.isinf(r_scan):
                continue

            current_angle = angle_min + (idx * angle_inc)
            angle_diff = abs(current_angle - person_theta)

            dist_sq = (person_dist**2)+(r_scan**2)-(2*person_dist*r_scan*math.cos(angle_diff))
            dist_person_to_point = math.sqrt(dist_sq)

            # 壁かどうかを検証
            if dist_person_to_point > 0.4:
                if dist_person_to_point < min_wall_dist:
                    min_wall_dist = dist_person_to_point
                    found_wall = True
        if found_wall:
            return min_wall_dist
        return None
    """

    def get_min_distance_to_sur(self, person_angle, person_dist, offset_rad=0.0):
        """
        人から最も近い壁からの距離を検出
        """
        if self.scan_ranges is None:
            return None

        # 人の角度を算出
        raw_angle = person_angle + offset_rad
        person_theta = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

        num_points = len(self.scan_ranges)
        lidar_angles = self.angle_min + (np.arange(num_points) * self.angle_inc)

        cos_diffs = np.cos(lidar_angles - person_theta)

        dist_sq = (person_dist**2) + (self.scan_ranges**2) - (2*person_dist*self.scan_ranges*cos_diffs)

        valid_mask = (self.scan_ranges > 0.1) & (np.isfinite(self.scan_ranges))

        valid_dist_sq = dist_sq[valid_mask]
        if len(valid_dist_sq) == 0:
            return None
        
        dists_to_points = np.sqrt(valid_dist_sq)
        wall_candidates = dists_to_points[dists_to_points > 0.5]

        if len(wall_candidates) > 0:
            return np.min(wall_candidates)
        return None

class LidarSystemRansac:
    def __init__(self):
        self.latest_scan = None
        self.scan_ranges = None
        self.angle_min = 0
        self.angle_inc = 0
        
        # Stores lines as (a, b, c) where ax+by+c=0
        self.wall_lines = []
        self.is_calibrated = False

    def update_scan(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.latest_scan = msg

        if not self.is_calibrated and len(self.scan_ranges) > 0:
            self.calibrate_walls()

    def calibrate_walls(self):
        """
        Runs a lightweight RANSAC
        """
        print("Calibrating walls on Pi...")

        # Prepare X, Y data
        angles = self.angle_min + np.arange(len(self.scan_ranges)) * self.angle_inc
        valid = (self.scan_ranges > 0.1) & (np.isfinite(self.scan_ranges))

        r = self.scan_ranges[valid]
        theta = angles[valid]

        X = r * np.cos(theta)
        Y = r * np.sin(theta)
        current_points = np.column_stack((X, Y))

        diffs = np.linalg.norm(current_points[:-1]-current_points[1:], axis=1)
        JUMP_THRESHOLD = 0.15
        split_indices = np.where(diffs > JUMP_THRESHOLD)[0]+1
        clusters = np.split(current_points, split_indices)

        
        self.wall_lines = []
        """
        for i in range(10):
            if len(current_points) < 20:
                break

            best_line, inlier_mask = self.fit_line_mask(current_points, threshold=0.10)
                
            if best_line is None:
                break
            self.wall_lines.append(best_line)
            current_points = current_points[~inlier_mask]
            print(f"Wall {i+1} found. Point remaining: {len(current_points)}")
        """
        for cluster in clusters:
            if len(cluster) < 15:
                continue
            width = np.linalg.norm(cluster[0]-cluster[-1])
            if width < 0.5:
                continue
            line, _  = self.fit_line_mask(cluster, iterations=20)
            if line:
                self.wall_lines.append(line)
        self.is_calibrated = True
        print(f"Calibration Done. Found {len(self.wall_lines)} walls")

    def get_distance_at_angle(self, angle_rad, offset_rad=0.0):
        """
        Calculates the closest valid distance within a small cone around the target angle.
        """
        if self.latest_scan is None:
            return None
        
        # 1. Calculate Target Angle
        raw_angle = angle_rad + offset_rad
        target_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        if angle_inc == 0:
            return None

        # 2. Convert Angle to Index
        center_idx = int(round((target_angle - angle_min) / angle_inc))
        num_ranges = len(scan.ranges)

        # 3. Define a "Search Cone" (e.g., +/- 3 degrees)
        # 3 degrees ~= 0.05 radians
        search_radius_rad = 0.02
        search_radius_idx = int(search_radius_rad / angle_inc)
        
        valid_distances = []

        # 4. Loop through indices in the cone
        for i in range(center_idx - search_radius_idx, center_idx + search_radius_idx + 1):
            # Handle wrap-around (e.g., -5 becomes 355)
            # This is crucial for 360 Lidar
            idx = i % num_ranges 
            
            dist = scan.ranges[idx]
            
            # Filter bad data (inf, nan, too close)
            # Turtlebot Lidar often returns 0.0 for errors
            if dist > 0.05 and not math.isinf(dist) and not math.isnan(dist):
                valid_distances.append(dist)

        # 5. Return the closest distance found in the cone
        if valid_distances:
            # We assume the closest point in the cone is the person
            min_dist = min(valid_distances)
            return min_dist, target_angle
            
        return None

    def fit_line_mask(self, points, iterations=100, threshold=0.1):
        """
        Returns:
            best_line: (a, b, c) tuple
            best_mask: Boolean array (True = point belongs to this wall)
        """
        n_points = len(points)
        best_inliers_count = 0
        best_line = None
        best_mask = None

        idx_pairs = np.random.randint(0, n_points, (iterations, 2))

        for idx1, idx2 in idx_pairs:
            p1 = points[idx1]
            p2 = points[idx2]

            if np.array_equal(p1, p2): continue

            a = p1[1] - p2[1]
            b = p2[0] - p1[0]
            c = -a * p1[0] - b * p1[1]

            norm = np.sqrt(a*a+b*b)
            if norm == 0: continue

            distances = np.abs(a * points[:, 0] + b * points[:, 1] +c)/norm

            current_mask = distances < threshold
            inliers_count = np.sum(current_mask)

            if inliers_count > best_inliers_count:
                best_inliers_count = inliers_count
                best_line = (a, b, c)
                best_mask = current_mask

        if best_inliers_count < 10:
            return None, None
        return best_line, best_mask

    def get_min_distance_to_sur(self, person_dist, person_angle, offset_rad=0.0):
        """
        Calculates distance from person to the closest saved wall line
        """
        if not self.is_calibrated or not self.wall_lines:
            return None

        # Person position
        raw_angle = person_angle + offset_rad
        px = person_dist * math.cos(raw_angle)
        py = person_dist * math.sin(raw_angle)
        min_dist = 999.0

        for (a, b, c) in self.wall_lines:
            norm = math.sqrt(a*a+b*b)
            dist = abs(a*px+b*py+c)/norm

            if dist < min_dist:
                min_dist = dist
        return min_dist

    def get_wall_markers(self, timestamp, frame_id):
        """
        creates and returns a MarkerArray representing the walls
        """
        if not self.is_calibrated or not self.wall_lines:
            return MarkerArray()
        marker_array = MarkerArray()

        for i, (a, b, c) in enumerate(self.wall_lines):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "walls"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0

            norm = math.sqrt(a*a+b*b)
            p0_x = -a*c/(norm**2)
            p0_y = -b*c/(norm**2)
            vec_x = -b/norm
            vec_y = a/norm
            length = 5.0

            p1 = Point(x=p0_x+(vec_x*length), y=p0_y+(vec_y*length), z=0.0)
            p2 = Point(x=p0_x-(vec_x*length), y=p0_y-(vec_y*length), z=0.0)
            marker.points.append(p1)
            marker.points.append(p2)
            marker_array.markers.append(marker)
        return marker_array

class LidarSystemHough:
    def __init__(self):
        self.latest_scan = None
        self.scan_ranges = None
        self.angle_min = 0
        self.angle_inc = 0

        self.wall_lines = []
        self.is_calibrated = False

        self.map_size_px = 1000
        self.map_resolution = 0.02
        self.map_center = self.map_size_px // 2

    def update_scan(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.latest_scan = msg

        if not self.is_calibrated and len(self.scan_ranges) > 0:
            self.calibrate_walls()

    def calibrate_walls(self):
        print("Calibrating walls with Hough Transform...")

        angles = self.angle_min + np.arange(len(self.scan_ranges)) * self.angle_inc
        valid = (self.scan_ranges > 0.1) & (np.isfinite(self.scan_ranges)) & (self.scan_ranges < 10.0)

        r = self.scan_ranges[valid]
        theta = angles[valid]

        X = r * np.cos(theta)
        Y = r * np.sin(theta)
        # image
        grid = np.zeros((self.map_size_px, self.map_size_px), dtype=np.uint8)
        # convert meters to pixels
        px_indices = (X / self.map_resolution).astype(int) + self.map_center
        py_indices = (Y / self.map_resolution).astype(int) + self.map_center

        # filter points outside the image
        mask = (px_indices >= 0) & (px_indices < self.map_size_px) & \
        (py_indices >= 0) & (py_indices < self.map_size_px)

        px_indices = px_indices[mask]
        py_indices = py_indices[mask]
        grid[py_indices, px_indices] = 255
        kernel = np.ones((3, 3), np.uint8)
        grid = cv2.dilate(grid, kernel, iterations=1)

        # hough transform
        min_pixels = int(1.5/self.map_resolution)
        gap_pixels = int(0.5/self.map_resolution)

        lines = cv2.HoughLinesP(grid, 3, np.pi/180, threshold=15,
        minLineLength=min_pixels, maxLineGap=gap_pixels)

        self.wall_lines = []

        if lines is not None:
            for line in lines:
                x1_px, y1_px, x2_px, y2_px = line[0]

                x1 = (x1_px - self.map_center) * self.map_resolution
                y1 = (y1_px - self.map_center) * self.map_resolution
                x2 = (x2_px - self.map_center) * self.map_resolution
                y2 = (y2_px - self.map_center) * self.map_resolution
                
                self.wall_lines.append((x1, y1, x2, y2))
        self.is_calibrated = True
        print(f"Hough Calibration Done. Found {len(self.wall_lines)} walls.")
        cv2.imwrite("/home/ubuntu/lidar_hough.png", grid)

    def get_wall_markers(self, timestamp, frame_id):
        if not self.wall_lines:
            return MarkerArray()
            
        marker_array = MarkerArray()
        
        for i, (x1, y1, x2, y2) in enumerate(self.wall_lines):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = timestamp
            marker.ns = "hough_walls"
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
            marker.lifetime.sec = 0
            
            p1 = Point(x=x1, y=y1, z=0.0)
            p2 = Point(x=x2, y=y2, z=0.0)
            
            marker.points.append(p1)
            marker.points.append(p2)
            marker_array.markers.append(marker)
            
        return marker_array

    def get_min_distance_to_sur(self, person_dist, person_angle, offset_rad=0.0):
        # Same logic as before: Vector distance to segments
        if not self.wall_lines: return None, None, None
        
        px = person_dist * math.cos(person_angle + offset_rad)
        py = person_dist * math.sin(person_angle + offset_rad)
        P = np.array([px, py])
        min_dist = 999.0
        closest_wall_point = None

        for (x1, y1, x2, y2) in self.wall_lines:
            A = np.array([x1, y1])
            B = np.array([x2, y2])
            
            # Distance from point P to segment AB
            AB = B - A
            AP = P - A
            length_sq = np.dot(AB, AB)
            if length_sq == 0: continue
            
            t = max(0.0, min(1.0, np.dot(AP, AB) / length_sq))
            closest = A + t * AB
            dist = np.linalg.norm(P - closest)
            
            if dist < min_dist:
                min_dist = dist
                closet_wall_point = closest
        return min_dist, P, closest_wall_point

    def get_distance_at_angle(self, angle_rad, offset_rad=0.0):
        """
        Calculates the closest valid distance within a small cone around the target angle.
        """
        if self.latest_scan is None:
            return None
        
        # 1. Calculate Target Angle
        raw_angle = angle_rad + offset_rad
        target_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        if angle_inc == 0:
            return None

        # 2. Convert Angle to Index
        center_idx = int(round((target_angle - angle_min) / angle_inc))
        num_ranges = len(scan.ranges)

        # 3. Define a "Search Cone" (e.g., +/- 3 degrees)
        # 3 degrees ~= 0.05 radians
        search_radius_rad = 0.02
        search_radius_idx = int(search_radius_rad / angle_inc)
        
        valid_distances = []

        # 4. Loop through indices in the cone
        for i in range(center_idx - search_radius_idx, center_idx + search_radius_idx + 1):
            # Handle wrap-around (e.g., -5 becomes 355)
            # This is crucial for 360 Lidar
            idx = i % num_ranges 
            
            dist = scan.ranges[idx]
            
            # Filter bad data (inf, nan, too close)
            # Turtlebot Lidar often returns 0.0 for errors
            if dist > 0.05 and not math.isinf(dist) and not math.isnan(dist):
                valid_distances.append(dist)

        # 5. Return the closest distance found in the cone
        if valid_distances:
            # We assume the closest point in the cone is the person
            min_dist = min(valid_distances)
            return min_dist, target_angle
            
        return None

