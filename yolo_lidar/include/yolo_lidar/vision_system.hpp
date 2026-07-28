#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// MNN headers
#include <MNN/Interpreter.hpp>
#include <MNN/MNNDefine.h>
#include <MNN/Tensor.hpp>

#include "sensor_msgs/msg/camera_info.hpp"

namespace yolo_lidar {

/// A single YOLO detection (before NMS).
struct Detection {
    int     class_id;
    float   confidence;
    cv::Rect bbox;  // pixel coords in the original (pre-letterbox) image
};

/// Wraps YOLO-MNN inference + ArUco detection + simple centroid tracking.
/// Ported from VisionSystem (vision.py).
class VisionSystem {
public:
    /// @param weights_path  Path to the .mnn weights file (e.g. "yolo26n.mnn")
    /// @param conf_thres    Minimum confidence to accept a detection
    /// @param imgsz         Square input size fed to YOLO (e.g. 640)
    VisionSystem(const std::string& weights_path, float conf_thres, int imgsz);
    ~VisionSystem();

    /// Store camera intrinsics; must be called before detectPeopleAngles.
    void setCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr msg);

    bool hasCameraInfo() const { return camera_info_ != nullptr; }

    /// Detect people in the image, returning {person_id → horizontal_angle_rad}
    /// and an annotated debug image.
    std::pair<std::map<int, double>, cv::Mat>
    detectPeopleAngles(const cv::Mat& bgr_image);

private:
    // ── YOLO / MNN ──────────────────────────────────────────────────────────

    /// Pre-process, run the MNN session, post-process (NMS), return detections.
    std::vector<Detection> runYolo(const cv::Mat& bgr_image);

    /// Letterbox-resize image to (imgsz_ x imgsz_) with grey padding.
    cv::Mat letterbox(const cv::Mat& src, float& scale,
                      int& pad_left, int& pad_top) const;

    /// Decode raw float output ([1, 84, 8400] YOLOv8 format) into Detections.
    std::vector<Detection> decodeOutput(
        const float* data, int num_anchors, int num_attrs,
        float scale, int pad_left, int pad_top,
        int orig_w, int orig_h) const;

    // ── ArUco ───────────────────────────────────────────────────────────────

    /// Detect ArUco markers; returns {marker_id → centre_point}.
    std::map<int, cv::Point2f> detectAruco(
        const cv::Mat& bgr_image,
        std::vector<std::vector<cv::Point2f>>& corners_out,
        std::vector<int>& ids_out) const;

    // ── Simple IoU tracker ──────────────────────────────────────────────────

    /// Match new detections to existing tracks; returns track_id per detection.
    std::vector<int> matchDetections(const std::vector<Detection>& dets);

    int assignNewTrackId() { return next_track_id_++; }

    static float computeIoU(const cv::Rect& a, const cv::Rect& b);
    static bool  pointInBox(float px, float py, const cv::Rect& box);

    // ── State ────────────────────────────────────────────────────────────────

    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_{nullptr};

    // MNN
    MNN::Interpreter* interpreter_{nullptr};
    MNN::Session*     session_{nullptr};
    MNN::Tensor*      input_tensor_{nullptr};
    float             conf_thres_;
    int               imgsz_;

    // ArUco
    cv::Ptr<cv::aruco::Dictionary>        aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    // Tracker
    struct TrackState {
        cv::Rect bbox;
        int      age{0};   // frames since last matched
    };
    std::map<int, TrackState> tracks_;         // track_id → state
    std::map<int, int>        track_to_aruco_; // track_id → aruco_id
    int next_track_id_{1};

    static constexpr int   YOLO_ID_OFFSET     = 10000;
    static constexpr int   MAX_TRACK_AGE      = 30;   // frames before expiry
    static constexpr float IOU_MATCH_THRESHOLD = 0.3f;
};

}  // namespace yolo_lidar
