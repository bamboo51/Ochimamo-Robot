#pragma once

#include <array>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace yolo_lidar {

/// A wall segment detected by the Hough transform, stored in metric coordinates.
struct WallSegment {
    double x1, y1, x2, y2;
};

/// Fuses LiDAR scan data with Hough-transform-based wall detection.
/// Ported from LidarSystemHough (lidar.py).
class LidarSystemHough {
public:
    LidarSystemHough();

    /// Ingest a new LaserScan message. Triggers wall calibration on first call.
    void updateScan(sensor_msgs::msg::LaserScan::SharedPtr msg);

    /// Returns {distance_m, corrected_angle_rad} for the nearest valid point
    /// within a small cone around (angle_rad + offset_rad), or nullopt.
    std::optional<std::pair<double, double>> getDistanceAtAngle(
        double angle_rad, double offset_rad = 0.0) const;

    /// Returns {min_dist_to_wall, person_xy, closest_wall_xy} or {-1, {}, {}}.
    std::tuple<double, std::array<double, 2>, std::array<double, 2>>
    getMinDistanceToSur(double person_dist, double person_angle,
                        double offset_rad = 0.0) const;

    /// Builds a MarkerArray visualising the detected wall segments.
    visualization_msgs::msg::MarkerArray getWallMarkers(
        const rclcpp::Time& timestamp, const std::string& frame_id) const;

    bool is_calibrated{false};
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan{nullptr};

private:
    void calibrateWalls();

    std::vector<float> scan_ranges_;
    double angle_min_{0.0};
    double angle_inc_{0.0};

    std::vector<WallSegment> wall_lines_;

    // Hough occupancy-grid parameters (match Python defaults)
    static constexpr int    MAP_SIZE_PX   = 1000;
    static constexpr double MAP_RESOLUTION = 0.02;   // metres per pixel
    static constexpr int    MAP_CENTER     = MAP_SIZE_PX / 2;
};

}  // namespace yolo_lidar
