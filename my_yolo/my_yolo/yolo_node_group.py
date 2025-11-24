import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.utilities import remove_ros_args
import numpy as np
from my_yolo.detector import Detector, parse_opt

def rosimg_to_cv2(msg):
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    except Exception as e:
        print(f"Image conversion error: {e}")
        return None
    
class ObjectDetection(Node):
    def __init__(self, enable_following=False, **args):
        super().__init__("object_detection")
        self.detector = Detector(**args)
        self.enable_following = enable_following

        if self.enable_following:
            self.declare_parameter("angular_kp", 0.005)
            self.declare_parameter("max_angular_vel", 0.05)
            self.declare_parameter("center_tolerance", 50)

            self.angular_kp = self.get_parameter("angular_kp").value
            self.max_angular_vel = self.get_parameter("max_angular_vel").value
            self.center_tolerance = self.get_parameter("center_tolerance").value

            self.img_width = None
            self.img_height = None
            self.img_center_x = None

            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.last_detection_time = self.get_clock().now()
            self.safety_timer = self.create_timer(0.5, self.safety_check)
            self.control_enabled = True
            self.get_logger().info("GROUP CENTERING mode ENABLED")

        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data
        )

    def stop_robot(self):
        if self.enable_following:
            self.publish_cmd_vel(0.0, 0.0)
    
    def safety_check(self):
        if not self.enable_following: return
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_detection > 1.0:
            self.stop_robot()

    def get_group_bounding_box(self, detections):
        """
        Calculates a bounding box that encompasses ALL detected people.
        Returns: (min_u, min_v, max_u, max_v, center_x)
        """
        if not detections: return None
        
        # Initialize with extreme values
        min_u = float('inf')
        min_v = float('inf')
        max_u = float('-inf')
        max_v = float('-inf')

        for det in detections:
            if det.u1 < min_u: min_u = det.u1
            if det.v1 < min_v: min_v = det.v1
            if det.u2 > max_u: max_u = det.u2
            if det.v2 > max_v: max_v = det.v2
            
        group_center_x = (min_u + max_u) / 2.0
        
        return (min_u, min_v, max_u, max_v, group_center_x)
    
    def calculate_control(self, group_center_x):
        if self.img_center_x is None: return 0.0, 0.0

        error_x = group_center_x - self.img_center_x

        # Angular (Steering)
        angular_z = -self.angular_kp * error_x
        angular_z = np.clip(angular_z, -self.max_angular_vel, self.max_angular_vel)

        # Deadband
        if abs(error_x) < self.center_tolerance:
            angular_z = 0.0

        return 0.0, angular_z # Linear X is always 0
    
    def publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def image_callback(self,msg):
        if self.enable_following and self.img_width is None:
            self.img_width = msg.height
            self.img_height = msg.width
            self.img_center_x = self.img_width / 2.0
            self.get_logger().info(f"Screen Configured. Center X: {self.img_center_x}")

        img0 = rosimg_to_cv2(msg)
        if img0 is None: return
        
        img_display, detections = self.detector.detect(img0)

        if self.enable_following:
            if detections:
                self.last_detection_time = self.get_clock().now()
                
                # --- CHANGED: Get Group Data instead of Single Person ---
                group_data = self.get_group_bounding_box(detections)
                # ------------------------------------------------------

                if group_data and self.control_enabled:
                    min_u, min_v, max_u, max_v, group_center_x = group_data
                    
                    linear_x, angular_z = self.calculate_control(group_center_x)
                    
                    count = len(detections)
                    self.get_logger().info(f"Centering {count} People | Rotation: {angular_z:.3f}")
                    
                    self.publish_cmd_vel(linear_x, angular_z)
                    
                    if self.detector.view_img:
                        # Draw Center Line
                        cv2.line(img_display, 
                                (int(self.img_center_x), 0), 
                                (int(self.img_center_x), int(self.img_height)), 
                                (0, 255, 255), 2)
                        
                        # Draw Group Box (Cyan)
                        cv2.rectangle(img_display, 
                                      (int(min_u), int(min_v)), 
                                      (int(max_u), int(max_v)), 
                                      (255, 255, 0), 4)
                        
                        # Draw Group Center Point
                        cv2.circle(img_display, 
                                   (int(group_center_x), int((min_v + max_v)/2)), 
                                   10, (255, 255, 0), -1)

            elif not detections:
                pass

        if self.detector.view_img:
            if self.enable_following:
                status = f"GROUP MODE ({len(detections)})" if self.control_enabled else "PAUSED"
                color = (0, 255, 0) if self.control_enabled else (0, 0, 255)
                cv2.putText(img_display, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            cv2.imshow("Person Detection", img_display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.stop_robot()
                rclpy.shutdown()
            elif key == ord('s') and self.enable_following:
                self.control_enabled = not self.control_enabled
                if not self.control_enabled: self.stop_robot()

def main():
    rclpy.init()
    enabled_following = "--follow" in sys.argv
    if enabled_following:
        sys.argv.remove("--follow")
    
    opt = parse_opt(remove_ros_args(args=sys.argv))
    opt_dict = vars(opt)
    opt_dict.pop("no_view", None)

    node = ObjectDetection(enable_following=enabled_following, **opt_dict)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_robot()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()