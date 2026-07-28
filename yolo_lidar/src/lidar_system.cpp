#include "yolo_lidar/lidar_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace yolo_lidar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

LidarSystemHough::LidarSystemHough() = default;

// ─────────────────────────────────────────────────────────────────────────────
// updateScan
// ─────────────────────────────────────────────────────────────────────────────

void LidarSystemHough::updateScan(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ranges_.assign(msg->ranges.begin(), msg->ranges.end());
    angle_min_ = msg->angle_min;
    angle_inc_ = msg->angle_increment;
    latest_scan = msg;

    if (!is_calibrated && !scan_ranges_.empty()) {
        calibrateWalls();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// calibrateWalls  (Hough transform on an occupancy grid)
// ─────────────────────────────────────────────────────────────────────────────

void LidarSystemHough::calibrateWalls()
{
    std::printf("[LidarSystemHough] Calibrating walls with Hough Transform...\n");

    const int    n    = static_cast<int>(scan_ranges_.size());
    const double aMin = angle_min_;
    const double aInc = angle_inc_;

    // Build binary occupancy grid
    cv::Mat grid = cv::Mat::zeros(MAP_SIZE_PX, MAP_SIZE_PX, CV_8UC1);

    for (int i = 0; i < n; ++i) {
        float r = scan_ranges_[i];
        if (r <= 0.1f || !std::isfinite(r) || r > 10.0f) {
            continue;
        }
        double theta = aMin + i * aInc;
        double mx    = r * std::cos(theta);
        double my    = r * std::sin(theta);

        int px = static_cast<int>(std::round(mx / MAP_RESOLUTION)) + MAP_CENTER;
        int py = static_cast<int>(std::round(my / MAP_RESOLUTION)) + MAP_CENTER;

        if (px >= 0 && px < MAP_SIZE_PX && py >= 0 && py < MAP_SIZE_PX) {
            grid.at<uint8_t>(py, px) = 255;
        }
    }

    // Dilate to connect nearby points
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::dilate(grid, grid, kernel, {-1, -1}, 1);

    // Probabilistic Hough line detection
    const int minPx  = static_cast<int>(1.5 / MAP_RESOLUTION);  // 1.5 m min line
    const int gapPx  = static_cast<int>(0.5 / MAP_RESOLUTION);  // 0.5 m max gap
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(grid, lines, /*rho=*/3, CV_PI / 180.0,
                    /*threshold=*/15, minPx, gapPx);

    wall_lines_.clear();
    for (const auto& l : lines) {
        double x1 = (l[0] - MAP_CENTER) * MAP_RESOLUTION;
        double y1 = (l[1] - MAP_CENTER) * MAP_RESOLUTION;
        double x2 = (l[2] - MAP_CENTER) * MAP_RESOLUTION;
        double y2 = (l[3] - MAP_CENTER) * MAP_RESOLUTION;
        wall_lines_.push_back({x1, y1, x2, y2});
    }

    is_calibrated = true;
    std::printf("[LidarSystemHough] Hough Calibration Done. Found %zu walls.\n",
                wall_lines_.size());

    // Save debug image (same path as Python version)
    cv::imwrite("/home/ubuntu/lidar_hough.png", grid);
}

// ─────────────────────────────────────────────────────────────────────────────
// getDistanceAtAngle
// ─────────────────────────────────────────────────────────────────────────────

std::optional<std::pair<double, double>>
LidarSystemHough::getDistanceAtAngle(double angle_rad, double offset_rad) const
{
    if (!latest_scan) {
        return std::nullopt;
    }

    // 1. Normalise target angle to [-pi, pi]
    double raw        = angle_rad + offset_rad;
    double target_ang = std::atan2(std::sin(raw), std::cos(raw));

    const double aMin = latest_scan->angle_min;
    const double aInc = latest_scan->angle_increment;
    if (aInc == 0.0) {
        return std::nullopt;
    }

    // 2. Convert angle to index
    int num_ranges  = static_cast<int>(latest_scan->ranges.size());
    int center_idx  = static_cast<int>(
        std::round((target_ang - aMin) / aInc));

    // 3. Search cone of ±0.02 rad
    constexpr double SEARCH_RAD = 0.02;
    int search_idx = static_cast<int>(SEARCH_RAD / aInc);

    float min_dist = std::numeric_limits<float>::infinity();
    bool  found    = false;

    for (int i = center_idx - search_idx; i <= center_idx + search_idx; ++i) {
        int idx  = ((i % num_ranges) + num_ranges) % num_ranges;  // wrap-around
        float d  = latest_scan->ranges[idx];
        if (d > 0.05f && std::isfinite(d)) {
            if (d < min_dist) {
                min_dist = d;
                found    = true;
            }
        }
    }

    if (found) {
        return std::make_pair(static_cast<double>(min_dist), target_ang);
    }
    return std::nullopt;
}

// ─────────────────────────────────────────────────────────────────────────────
// getMinDistanceToSur  (vector distance from person to nearest wall segment)
// ─────────────────────────────────────────────────────────────────────────────

std::tuple<double, std::array<double, 2>, std::array<double, 2>>
LidarSystemHough::getMinDistanceToSur(double person_dist, double person_angle,
                                       double offset_rad) const
{
    constexpr double NOT_FOUND = -1.0;

    if (wall_lines_.empty()) {
        return {NOT_FOUND, {}, {}};
    }

    double px = person_dist * std::cos(person_angle + offset_rad);
    double py = person_dist * std::sin(person_angle + offset_rad);

    double min_dist  = std::numeric_limits<double>::max();
    std::array<double, 2> closest_wall_pt{};
    bool found = false;

    for (const auto& seg : wall_lines_) {
        // Vector projection of P onto segment AB, clamped to [0,1]
        double ax = seg.x1, ay = seg.y1;
        double bx = seg.x2, by = seg.y2;

        double abx = bx - ax, aby = by - ay;
        double apx = px - ax, apy = py - ay;
        double len_sq = abx * abx + aby * aby;

        if (len_sq == 0.0) {
            continue;
        }

        double t = std::max(0.0, std::min(1.0,
            (apx * abx + apy * aby) / len_sq));

        double cx = ax + t * abx;
        double cy = ay + t * aby;

        double dist = std::hypot(px - cx, py - cy);
        if (dist < min_dist) {
            min_dist         = dist;
            closest_wall_pt  = {cx, cy};
            found            = true;
        }
    }

    if (!found) {
        return {NOT_FOUND, {}, {}};
    }

    return {min_dist, {px, py}, closest_wall_pt};
}

// ─────────────────────────────────────────────────────────────────────────────
// getWallMarkers
// ─────────────────────────────────────────────────────────────────────────────

visualization_msgs::msg::MarkerArray
LidarSystemHough::getWallMarkers(const rclcpp::Time& timestamp,
                                  const std::string& frame_id) const
{
    visualization_msgs::msg::MarkerArray array;
    if (wall_lines_.empty()) {
        return array;
    }

    int id = 0;
    for (const auto& seg : wall_lines_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id    = frame_id;
        m.header.stamp       = timestamp;
        m.ns                 = "hough_walls";
        m.id                 = id++;
        m.type               = visualization_msgs::msg::Marker::LINE_LIST;
        m.action             = visualization_msgs::msg::Marker::ADD;
        m.scale.x            = 0.05;
        m.color.r            = 0.0f;
        m.color.g            = 1.0f;
        m.color.b            = 0.0f;
        m.color.a            = 1.0f;
        m.lifetime.sec       = 0;
        m.lifetime.nanosec   = 0;

        geometry_msgs::msg::Point p1, p2;
        p1.x = seg.x1;  p1.y = seg.y1;  p1.z = 0.0;
        p2.x = seg.x2;  p2.y = seg.y2;  p2.z = 0.0;
        m.points.push_back(p1);
        m.points.push_back(p2);

        array.markers.push_back(m);
    }

    return array;
}

}  // namespace yolo_lidar
