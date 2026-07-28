#pragma once

#include <array>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "yolo_lidar/lidar_system.hpp"
#include "yolo_lidar/vision_system.hpp"

namespace yolo_lidar {

class PeopleMapperNode : public rclcpp::Node {
public:
    PeopleMapperNode();

private:
    // ── Callbacks ────────────────────────────────────────────────────────────
    void wallPubCallback();
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);

    // ── Helpers ───────────────────────────────────────────────────────────────
    void publishDistVector(const std::array<double, 2>& person_pt,
                           const std::array<double, 2>& wall_pt);

    visualization_msgs::msg::Marker
    createMarker(const geometry_msgs::msg::PointStamped& pt_map, int m_id) const;

    // ── Core systems ─────────────────────────────────────────────────────────
    LidarSystemHough lidar_;
    VisionSystem     vision_;

    double      cam_lidar_yaw_;
    std::string target_frame_;

    // ── TF2 ──────────────────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Publishers ────────────────────────────────────────────────────────────
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              debug_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                wall_dist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      vector_pub_;

    // ── Subscribers ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr  camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       image_sub_;

    // ── Timer ────────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr wall_timer_;
};

}  // namespace yolo_lidar
