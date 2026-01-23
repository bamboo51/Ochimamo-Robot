import numpy as np
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .vision import VisionSystem
from .lidar import LidarSystemHough as LidarSystem

import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

class PeopleMapperNode(Node):
    def __init__(self, weights, imgsz, conf_thres):
        super().__init__("people_mapper")

        self.vision = VisionSystem(weights, conf_thres, imgsz)
        self.lidar = LidarSystem()

        self.declare_parameter("camera_lidar_yaw", math.pi/2-0.07*math.pi) # +3.1415926 for backward camera
        self.declare_parameter("target_frame", "odom") #? how to change to map??? 
        self.cam_lidar_yaw = self.get_parameter("camera_lidar_yaw").value
        self.target_frame = self.get_parameter("target_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, "/people_markers", 10)
        self.debug_image_pub = self.create_publisher(Image, "/people_mapper/debug_image", 10)
        self.wall_dist_pub = self.create_publisher(String, "/people/wall_distance", 10)
        self.wall_pub = self.create_publisher(MarkerArray, "/wall_markers", 10)
        self.vector_pub = self.create_publisher(Marker, 'distance_vector', 10)

        self.create_subscription(LaserScan, "/scan", self.lidar.update_scan, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.vision.set_camera_info, 10)
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data)
        
        self.create_timer(0.1, self.wall_pub_func)

        self.get_logger().info("People Mapper Started.")

    def wall_pub_func(self):
        if self.lidar.is_calibrated and self.lidar.latest_scan:
            # now = self.get_clock().now().to_msg()
            frame = self.lidar.latest_scan.header.frame_id
            wall_markers = self.lidar.get_wall_markers(self.lidar.latest_scan.header.stamp, frame)
            self.wall_pub.publish(wall_markers)

    def publish_dist_vector(self, publisher_node, person_point, wall_point):
        """
        Draws a vector from the person to the wall
        """
        if person_point is None or wall_point is None:
            return

        marker = Marker()
        marker.header.frame_id = self.lidar.latest_scan.header.frame_id
        marker.header.stamp = self.lidar.latest_scan.header.stamp
        marker.ns = "distance_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # arrow dimensions
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000

        p_start = Point(x=float(person_point[0]), y=float(person_point[1]), z=0.0)
        p_end = Point(x=float(wall_point[0]), y=float(wall_point[1]), z=0.0)
        marker.points.append(p_start)
        marker.points.append(p_end)
        publisher_node.publish(marker)

    def imgmsg_to_cv2(self, msg):
        """
        ROSメッセージからCV2に変換
        """
        dtype = np.uint8
        n_channels = 3

        if "rgb8" in msg.encoding or "bgr8" in msg.encoding:
            n_channels = 3
        elif "mono8" in msg.encoding:
            n_channels = 1

        img_buf = np.frombuffer(msg.data, dtype=dtype)
        img = img_buf.reshape(msg.height, msg.width, n_channels)

        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        """
        CV2からROSメッセージへ
        """
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]

        if len(cv_image.shape) == 3:
            msg.encoding = "bgr8"
            msg.step = cv_image.shape[1] * 3
        else:
            msg.encoding = "mono8"
            msg.step = cv_image.shape[1]

        msg.data = cv_image.tobytes()
        return msg

    def image_callback(self, msg):
        try:
            #img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            #if msg.encoding == "rgb-8":
                #img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
            img_np = self.imgmsg_to_cv2(msg)
        except Exception:
            return
        
        img_np = cv2.rotate(img_np, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.get_logger().info("Image received")
        
        # 角度を計算
        people_dict, annotated_frame= self.vision.detect_people_angles(img_np)
        if self.debug_image_pub.get_subscription_count() > 0:
            msg = self.cv2_to_imgmsg(annotated_frame)
            self.debug_image_pub.publish(img_np)
        if not people_dict:
            self.get_logger().info("No peopled detected")
            return

        if self.lidar.latest_scan is None:
            self.get_logger().warn("No LiDAR data yet!")
            return
        
        # 距離を取得
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        scan_time = self.lidar.latest_scan.header.stamp
        scan_frame_id = self.lidar.latest_scan.header.frame_id
        m_id = 0

        for track_id, theta in people_dict.items():
            result = self.lidar.get_distance_at_angle(theta, self.cam_lidar_yaw)

            if result:
                dist, final_angle = result
                self.get_logger().info(f"Person ID {track_id} is at {dist:.2f}m")

                # 人と壁の距離を計算
                nearest_wall, p_coords, w_coords = self.lidar.get_min_distance_to_sur(theta, dist, self.cam_lidar_yaw)
                if nearest_wall:
                    self.get_logger().info(f"Person {track_id}. Nearest wall is {nearest_wall}")

                    msg = String()
                    msg.data = f"{track_id}:{nearest_wall:.2f}" 
                    self.wall_dist_pub.publish(msg)
                    self.publish_dist_vec(self, p_coords, w_coords)


                # 角度座標→直径座標
                x_l = dist * math.cos(final_angle)
                y_l = dist * math.sin(final_angle)

                # LiDARフレーム→地図フレーム
                pt_lidar = PointStamped()
                pt_lidar.header.stamp = rclpy.time.Time().to_msg()
                # pt_lidar.header.stamp = scan_time
                pt_lidar.header.frame_id = self.lidar.latest_scan.header.frame_id
                pt_lidar.point.x = x_l
                pt_lidar.point.y = y_l

                try:
                    pt_map = self.tf_buffer.transform(
                        pt_lidar,
                        self.target_frame,
                        timeout=Duration(seconds=1.0)
                    )

                    marker = self.create_marker(pt_map, m_id)
                    marker_array.markers.append(marker)
                    m_id += 1
                except Exception as e:
                    self.get_logger().error(f"TF Transform Failed: {e}")
            else:
                self.get_logger().warn("Angle matched no valid LiDAR range.")

        self.marker_pub.publish(marker_array)

    def create_marker(self, point_map, m_id):
        marker = Marker()
        marker.header = point_map.header
        marker.ns = "people"
        marker.id = m_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point_map.point
        marker.pose.orientation.w = 1.0 # Important to prevent RViz warnings
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.r = 1.0; marker.color.a = 1.0
        
        # Marker lifetime (Python duration -> Message duration)
        # 0.5 seconds is good for smooth tracking
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000 
        return marker

def main():
    rclpy.init()
    # Hardcoded args for Pi 4 performance
    node = PeopleMapperNode(weights="yolo26n.mnn", imgsz=640, conf_thres=0.9)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
