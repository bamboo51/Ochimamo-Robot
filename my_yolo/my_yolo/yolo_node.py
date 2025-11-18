import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from rclpy.utilities import remove_ros_args
import numpy as np
from my_yolo.detector import Detector, parse_opt


def rosimg_to_cv2(msg):
    """Convert ROS Image message to OpenCV format"""
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    except Exception as e:
        print(f"Image conversion error: {e}")
        return None


class ObjectDetection(Node):
    def __init__(self, **args):
        super().__init__("object_detection")
        self.detector = Detector(**args)
        self.detection_results = []
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info("Person detection node started")

    def image_callback(self, msg):
        img0 = rosimg_to_cv2(msg)
        if img0 is None:
            self.get_logger().warn("Failed to convert image")
            return

        img_display, detections = self.detector.detect(img0)

        # Display results
        if self.detector.view_img:
            cv2.imshow("Person Detection", img_display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("'q' pressed, shutting down...")
                rclpy.shutdown()

        # Store and log detections
        if detections:
            self.detection_results.extend(detections)
            self.get_logger().info(f"Frame detections: {len(detections)}")
            for i, r in enumerate(detections):
                # Include tracking ID in log output
                id_str = f" ID={r.track_id}" if hasattr(r, 'track_id') and r.track_id is not None else ""
                self.get_logger().info(
                    f"  [{i}] bbox=[{r.u1:.1f}, {r.v1:.1f}, {r.u2:.1f}, {r.v2:.1f}] "
                    f"class='{r.name}' conf={r.conf:.3f}{id_str}"
                )


def main():
    rclpy.init()
    opt = parse_opt(remove_ros_args(args=sys.argv))
    opt_dict = vars(opt)
    opt_dict.pop('no_view', None)  # Remove if exists
    node = ObjectDetection(**opt_dict)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()