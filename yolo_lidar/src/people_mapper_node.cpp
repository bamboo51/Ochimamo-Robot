#include "yolo_lidar/people_mapper_node.hpp"

#include <cmath>
#include <chrono>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace yolo_lidar {

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

PeopleMapperNode::PeopleMapperNode()
    : Node("people_mapper"),
      // Hardcoded args matching Python version (for Pi 4 performance)
      vision_("yolo26n.mnn", /*conf_thres=*/0.9f, /*imgsz=*/640)
{
    // ── Parameters ──────────────────────────────────────────────────────────
    this->declare_parameter("camera_lidar_yaw",
        M_PI / 2.0 - 0.07 * M_PI);  // +M_PI for backward camera
    this->declare_parameter("target_frame", std::string("odom"));

    cam_lidar_yaw_ = this->get_parameter("camera_lidar_yaw").as_double();
    target_frame_  = this->get_parameter("target_frame").as_string();

    // ── TF2 ──────────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── Publishers ────────────────────────────────────────────────────────────
    marker_pub_    = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/people_markers", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/people_mapper/debug_image", 10);
    wall_dist_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/people/wall_distance", 10);
    wall_pub_      = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/wall_markers", 10);
    vector_pub_    = this->create_publisher<visualization_msgs::msg::Marker>(
        "distance_vector", 10);

    // ── Subscribers ───────────────────────────────────────────────────────────
    auto sensor_qos = rclcpp::SensorDataQoS();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            lidar_.updateScan(msg);
        });

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 10,
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            vision_.setCameraInfo(msg);
        });

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", sensor_qos,
        std::bind(&PeopleMapperNode::imageCallback, this, std::placeholders::_1));

    // ── Timer (100 ms wall publisher) ─────────────────────────────────────────
    wall_timer_ = this->create_wall_timer(
        100ms, std::bind(&PeopleMapperNode::wallPubCallback, this));

    RCLCPP_INFO(this->get_logger(), "People Mapper Started.");
}

// ─────────────────────────────────────────────────────────────────────────────
// wallPubCallback
// ─────────────────────────────────────────────────────────────────────────────

void PeopleMapperNode::wallPubCallback()
{
    if (lidar_.is_calibrated && lidar_.latest_scan) {
        const auto& header = lidar_.latest_scan->header;
        auto wall_markers  = lidar_.getWallMarkers(
            rclcpp::Time(header.stamp), header.frame_id);
        wall_pub_->publish(wall_markers);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// imageCallback
// ─────────────────────────────────────────────────────────────────────────────

void PeopleMapperNode::imageCallback(sensor_msgs::msg::Image::SharedPtr msg)
{
    // ── Decode ROS image → OpenCV ────────────────────────────────────────────
    cv::Mat img_np;
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        img_np      = cv_ptr->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "cv_bridge exception: %s", e.what());
        return;
    }

    // Rotate 90° CCW to match Python: cv2.ROTATE_90_COUNTERCLOCKWISE
    cv::rotate(img_np, img_np, cv::ROTATE_90_COUNTERCLOCKWISE);

    RCLCPP_INFO(this->get_logger(), "Image received");

    // ── Person detection ─────────────────────────────────────────────────────
    auto [people_dict, annotated_frame] = vision_.detectPeopleAngles(img_np);

    if (debug_image_pub_->get_subscription_count() > 0) {
        auto debug_msg = cv_bridge::CvImage(
            msg->header, "bgr8", annotated_frame).toImageMsg();
        debug_image_pub_->publish(*debug_msg);
    }

    if (people_dict.empty()) {
        RCLCPP_INFO(this->get_logger(), "No people detected");
        return;
    }

    if (!lidar_.latest_scan) {
        RCLCPP_WARN(this->get_logger(), "No LiDAR data yet!");
        return;
    }

    // ── Build MarkerArray for all detected people ────────────────────────────
    visualization_msgs::msg::MarkerArray marker_array;
    int m_id = 0;

    for (const auto& [track_id, theta] : people_dict) {
        auto result = lidar_.getDistanceAtAngle(theta, cam_lidar_yaw_);
        if (!result) {
            RCLCPP_WARN(this->get_logger(),
                        "Angle matched no valid LiDAR range.");
            continue;
        }

        auto [dist, final_angle] = *result;
        RCLCPP_INFO(this->get_logger(),
                    "Person ID %d is at %.2f m", track_id, dist);

        // ── Wall distance ────────────────────────────────────────────────────
        auto [nearest_wall, p_coords, w_coords] =
            lidar_.getMinDistanceToSur(dist, theta, cam_lidar_yaw_);

        if (nearest_wall > 0.0) {
            RCLCPP_INFO(this->get_logger(),
                        "Person %d. Nearest wall is %.2f", track_id, nearest_wall);

            std_msgs::msg::String wall_msg;
            wall_msg.data = std::to_string(track_id) + ":" +
                            std::to_string(nearest_wall);
            wall_dist_pub_->publish(wall_msg);

            publishDistVector(p_coords, w_coords);
        }

        // ── Polar → Cartesian (LiDAR frame) ─────────────────────────────────
        double x_l = dist * std::cos(final_angle);
        double y_l = dist * std::sin(final_angle);

        geometry_msgs::msg::PointStamped pt_lidar;
        pt_lidar.header.stamp    = rclcpp::Time(0);  // latest available tf
        pt_lidar.header.frame_id = lidar_.latest_scan->header.frame_id;
        pt_lidar.point.x         = x_l;
        pt_lidar.point.y         = y_l;
        pt_lidar.point.z         = 0.0;

        // ── TF2: LiDAR frame → target frame (odom) ──────────────────────────
        try {
            auto pt_map = tf_buffer_->transform(
                pt_lidar, target_frame_,
                tf2::durationFromSec(1.0));

            auto m = createMarker(pt_map, m_id++);
            marker_array.markers.push_back(m);
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "TF Transform Failed: %s", e.what());
        }
    }

    marker_pub_->publish(marker_array);
}

// ─────────────────────────────────────────────────────────────────────────────
// publishDistVector
// ─────────────────────────────────────────────────────────────────────────────

void PeopleMapperNode::publishDistVector(const std::array<double, 2>& person_pt,
                                          const std::array<double, 2>& wall_pt)
{
    if (!lidar_.latest_scan) {
        return;
    }

    visualization_msgs::msg::Marker m;
    m.header.frame_id  = lidar_.latest_scan->header.frame_id;
    m.header.stamp     = lidar_.latest_scan->header.stamp;
    m.ns               = "distance_vector";
    m.id               = 0;
    m.type             = visualization_msgs::msg::Marker::ARROW;
    m.action           = visualization_msgs::msg::Marker::ADD;

    // Arrow shaft / head dimensions
    m.scale.x          = 0.05;
    m.scale.y          = 0.1;
    m.scale.z          = 0.1;

    m.color.r          = 0.0f;
    m.color.g          = 1.0f;
    m.color.b          = 1.0f;
    m.color.a          = 1.0f;

    m.lifetime.sec     = 0;
    m.lifetime.nanosec = 200'000'000;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = person_pt[0];  p_start.y = person_pt[1];  p_start.z = 0.0;
    p_end.x   = wall_pt[0];    p_end.y   = wall_pt[1];    p_end.z   = 0.0;
    m.points.push_back(p_start);
    m.points.push_back(p_end);

    vector_pub_->publish(m);
}

// ─────────────────────────────────────────────────────────────────────────────
// createMarker
// ─────────────────────────────────────────────────────────────────────────────

visualization_msgs::msg::Marker
PeopleMapperNode::createMarker(const geometry_msgs::msg::PointStamped& pt_map,
                                int m_id) const
{
    visualization_msgs::msg::Marker m;
    m.header           = pt_map.header;
    m.ns               = "people";
    m.id               = m_id;
    m.type             = visualization_msgs::msg::Marker::SPHERE;
    m.action           = visualization_msgs::msg::Marker::ADD;
    m.pose.position    = pt_map.point;
    m.pose.orientation.w = 1.0;  // Prevents RViz warnings

    m.scale.x          = 0.3;
    m.scale.y          = 0.3;
    m.scale.z          = 0.3;

    m.color.r          = 1.0f;
    m.color.a          = 1.0f;

    // 0.5-second lifetime for smooth tracking
    m.lifetime.sec     = 0;
    m.lifetime.nanosec = 500'000'000;

    return m;
}

}  // namespace yolo_lidar
