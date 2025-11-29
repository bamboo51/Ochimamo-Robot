import math

class LidarSystem:
    def __init__(self):
        self.latest_scan = None

    def update_scan(self, msg):
        self.latest_scan = msg

    def get_distance_at_angle(self, angle_rad, offset_rad=0.0):
        """
        Calculates the closest valid distance within a small cone around the target angle.
        """
        if self.latest_scan is None:
            return None
        
        raw_angle = angle_rad + offset_rad
        target_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        
        if angle_inc == 0:
            return None

        center_idx = int(round((target_angle - angle_min) / angle_inc))
        num_ranges = len(scan.ranges)

        search_radius_rad = 0.05 
        search_radius_idx = int(search_radius_rad / angle_inc)
        
        valid_distances = []

        for i in range(center_idx - search_radius_idx, center_idx + search_radius_idx + 1):
            idx = i % num_ranges 
            
            dist = scan.ranges[idx]
            
            if dist > 0.05 and not math.isinf(dist) and not math.isnan(dist):
                valid_distances.append(dist)

        if valid_distances:
            min_dist = min(valid_distances)
            return min_dist, target_angle
            
        return None