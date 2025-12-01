import math
import numpy as np

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