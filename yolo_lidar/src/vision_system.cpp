#include "yolo_lidar/vision_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace yolo_lidar {

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

VisionSystem::VisionSystem(const std::string& weights_path,
                           float conf_thres, int imgsz)
    : conf_thres_(conf_thres), imgsz_(imgsz)
{
    // ── Load MNN model ───────────────────────────────────────────────────────
    interpreter_ = MNN::Interpreter::createFromFile(weights_path.c_str());
    if (!interpreter_) {
        throw std::runtime_error(
            "[VisionSystem] Failed to load MNN model: " + weights_path);
    }

    MNN::ScheduleConfig config;
    config.numThread = 4;
    config.type      = MNN_FORWARD_CPU;

    session_ = interpreter_->createSession(config);
    if (!session_) {
        throw std::runtime_error("[VisionSystem] MNN createSession() failed.");
    }

    input_tensor_ = interpreter_->getSessionInput(session_, nullptr);
    if (!input_tensor_) {
        throw std::runtime_error("[VisionSystem] Could not get MNN input tensor.");
    }

    // Pre-set input shape: NCHW = [1, 3, imgsz, imgsz]
    interpreter_->resizeTensor(input_tensor_, {1, 3, imgsz_, imgsz_});
    interpreter_->resizeSession(session_);

    // ── ArUco detector ──────────────────────────────────────────────────────
    aruco_dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();
}

VisionSystem::~VisionSystem()
{
    if (interpreter_ && session_) {
        interpreter_->releaseSession(session_);
    }
    MNN::Interpreter::destroy(interpreter_);
}

// ─────────────────────────────────────────────────────────────────────────────
// setCameraInfo
// ─────────────────────────────────────────────────────────────────────────────

void VisionSystem::setCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    camera_info_ = msg;
}

// ─────────────────────────────────────────────────────────────────────────────
// detectPeopleAngles  (top-level entry point, mirrors Python version)
// ─────────────────────────────────────────────────────────────────────────────

std::pair<std::map<int, double>, cv::Mat>
VisionSystem::detectPeopleAngles(const cv::Mat& bgr_image)
{
    // 1) ArUco detection
    std::vector<std::vector<cv::Point2f>> ar_corners;
    std::vector<int>                      ar_ids;
    auto markers = detectAruco(bgr_image, ar_corners, ar_ids);

    // 2) YOLO inference
    auto detections = runYolo(bgr_image);

    // 3) Simple tracker — assign track IDs to detections
    auto track_ids = matchDetections(detections);

    // 4) Build annotated debug image
    cv::Mat annotated = bgr_image.clone();
    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        cv::rectangle(annotated, det.bbox, {0, 255, 0}, 2);
        std::string label = "p:" + std::to_string(track_ids[i]) +
                            " " + std::to_string(det.confidence).substr(0, 4);
        cv::putText(annotated, label,
                    {det.bbox.x, det.bbox.y - 5},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
    }
    if (!ar_ids.empty() && !ar_corners.empty()) {
        cv::aruco::drawDetectedMarkers(annotated, ar_corners, ar_ids);
    }

    if (!camera_info_) {
        return {{}, annotated};
    }

    const double fx = camera_info_->k[0];
    const double cx = camera_info_->k[2];

    // 5) Associate ArUco markers to YOLO person boxes
    for (auto& [aruco_id, centre] : markers) {
        int   best_track   = -1;
        float best_area    = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (pointInBox(centre.x, centre.y, detections[i].bbox)) {
                float area = static_cast<float>(detections[i].bbox.area());
                if (area < best_area) {
                    best_area  = area;
                    best_track = track_ids[i];
                }
            }
        }
        if (best_track >= 0) {
            track_to_aruco_[best_track] = aruco_id;
        }
    }

    // 6) Build output map  {person_id → theta}
    std::map<int, double> people_dict;
    for (std::size_t i = 0; i < detections.size(); ++i) {
        const auto& bbox     = detections[i].bbox;
        double      u_center = bbox.x + bbox.width / 2.0;
        double      theta    = std::atan2(u_center - cx, fx);

        int track_id    = track_ids[i];
        int person_key  = track_to_aruco_.count(track_id)
                          ? track_to_aruco_.at(track_id)
                          : YOLO_ID_OFFSET + track_id;
        people_dict[person_key] = theta;
    }

    return {people_dict, annotated};
}

// ─────────────────────────────────────────────────────────────────────────────
// runYolo  (letterbox → MNN inference → decode → NMS)
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat VisionSystem::letterbox(const cv::Mat& src, float& scale,
                                int& pad_left, int& pad_top) const
{
    int orig_w = src.cols, orig_h = src.rows;
    float ratio = std::min(
        static_cast<float>(imgsz_) / orig_w,
        static_cast<float>(imgsz_) / orig_h);
    scale = ratio;

    int new_w = static_cast<int>(std::round(orig_w * ratio));
    int new_h = static_cast<int>(std::round(orig_h * ratio));

    cv::Mat resized;
    cv::resize(src, resized, {new_w, new_h}, 0, 0, cv::INTER_LINEAR);

    // Pad to (imgsz_ x imgsz_) with grey (114,114,114)
    pad_left = (imgsz_ - new_w) / 2;
    pad_top  = (imgsz_ - new_h) / 2;
    int pad_right  = imgsz_ - new_w - pad_left;
    int pad_bottom = imgsz_ - new_h - pad_top;

    cv::Mat out;
    cv::copyMakeBorder(resized, out,
                       pad_top, pad_bottom, pad_left, pad_right,
                       cv::BORDER_CONSTANT, {114, 114, 114});
    return out;
}

std::vector<Detection> VisionSystem::decodeOutput(
    const float* data, int num_anchors, int num_attrs,
    float scale, int pad_left, int pad_top,
    int orig_w, int orig_h) const
{
    // YOLOv8 output layout: [num_attrs, num_anchors] = [84, 8400]
    // For anchor i: x = data[0*num_anchors + i], y = data[1*num_anchors + i], ...
    // class scores start at offset 4*num_anchors

    std::vector<Detection> dets;
    const int num_classes = num_attrs - 4;

    for (int i = 0; i < num_anchors; ++i) {
        // Find best class score (person = class 0)
        float best_score = 0.0f;
        int   best_cls   = -1;
        for (int c = 0; c < num_classes; ++c) {
            float score = data[(4 + c) * num_anchors + i];
            if (score > best_score) {
                best_score = score;
                best_cls   = c;
            }
        }

        if (best_cls != 0 || best_score < conf_thres_) {
            continue;  // only keep person (class 0)
        }

        // Decode centre-xywh (in letterboxed space)
        float cx = data[0 * num_anchors + i];
        float cy = data[1 * num_anchors + i];
        float bw = data[2 * num_anchors + i];
        float bh = data[3 * num_anchors + i];

        // Convert to original image space
        float x1_lb = cx - bw / 2.0f;
        float y1_lb = cy - bh / 2.0f;
        float x2_lb = cx + bw / 2.0f;
        float y2_lb = cy + bh / 2.0f;

        float x1 = (x1_lb - pad_left) / scale;
        float y1 = (y1_lb - pad_top ) / scale;
        float x2 = (x2_lb - pad_left) / scale;
        float y2 = (y2_lb - pad_top ) / scale;

        // Clamp to image
        x1 = std::max(0.0f, std::min(x1, static_cast<float>(orig_w)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(orig_h)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(orig_w)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(orig_h)));

        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        Detection d;
        d.class_id  = best_cls;
        d.confidence = best_score;
        d.bbox       = cv::Rect(
            static_cast<int>(x1), static_cast<int>(y1),
            static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
        dets.push_back(d);
    }

    // NMS (OpenCV built-in)
    std::vector<cv::Rect>  boxes;
    std::vector<float>     scores;
    for (auto& d : dets) {
        boxes.push_back(d.bbox);
        scores.push_back(d.confidence);
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, scores, conf_thres_, /*nms_threshold=*/0.45f,
                      nms_indices);

    std::vector<Detection> result;
    result.reserve(nms_indices.size());
    for (int idx : nms_indices) {
        result.push_back(dets[idx]);
    }
    return result;
}

std::vector<Detection> VisionSystem::runYolo(const cv::Mat& bgr_image)
{
    if (!interpreter_ || !session_ || !input_tensor_) {
        return {};
    }

    float scale;
    int   pad_left, pad_top;
    cv::Mat lb = letterbox(bgr_image, scale, pad_left, pad_top);

    // BGR → RGB, float32, normalise to [0,1], HWC → CHW
    cv::Mat rgb;
    cv::cvtColor(lb, rgb, cv::COLOR_BGR2RGB);
    cv::Mat fp32;
    rgb.convertTo(fp32, CV_32FC3, 1.0 / 255.0);

    // Copy CHW into MNN input tensor
    auto* host = input_tensor_->host<float>();
    const int hw = imgsz_ * imgsz_;
    for (int h = 0; h < imgsz_; ++h) {
        for (int w = 0; w < imgsz_; ++w) {
            const cv::Vec3f& px = fp32.at<cv::Vec3f>(h, w);
            host[0 * hw + h * imgsz_ + w] = px[0];  // R
            host[1 * hw + h * imgsz_ + w] = px[1];  // G
            host[2 * hw + h * imgsz_ + w] = px[2];  // B
        }
    }

    interpreter_->runSession(session_);

    // Get output (shape: [1, 84, 8400] for YOLOv8 COCO)
    auto* out_tensor = interpreter_->getSessionOutput(session_, nullptr);
    if (!out_tensor) {
        return {};
    }

    // Copy to host
    auto* out_host = out_tensor->host<float>();

    // Determine layout: expect [1, num_attrs, num_anchors]
    const auto& dims = out_tensor->shape();
    // dims[0]=1, dims[1]=num_attrs, dims[2]=num_anchors
    int num_attrs   = (dims.size() >= 2) ? dims[1] : 84;
    int num_anchors = (dims.size() >= 3) ? dims[2] : 8400;

    return decodeOutput(out_host, num_anchors, num_attrs,
                        scale, pad_left, pad_top,
                        bgr_image.cols, bgr_image.rows);
}

// ─────────────────────────────────────────────────────────────────────────────
// detectAruco
// ─────────────────────────────────────────────────────────────────────────────

std::map<int, cv::Point2f> VisionSystem::detectAruco(
    const cv::Mat& bgr_image,
    std::vector<std::vector<cv::Point2f>>& corners_out,
    std::vector<int>& ids_out) const
{
    cv::Mat gray;
    cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, aruco_dict_, corners_out, ids_out,
                             aruco_params_, rejected);

    std::map<int, cv::Point2f> markers;
    for (std::size_t i = 0; i < ids_out.size(); ++i) {
        cv::Point2f centre{0, 0};
        for (const auto& pt : corners_out[i]) {
            centre += pt;
        }
        centre *= 0.25f;  // average of 4 corners
        markers[ids_out[i]] = centre;
    }
    return markers;
}

// ─────────────────────────────────────────────────────────────────────────────
// Simple IoU tracker
// ─────────────────────────────────────────────────────────────────────────────

float VisionSystem::computeIoU(const cv::Rect& a, const cv::Rect& b)
{
    cv::Rect inter = a & b;
    if (inter.empty()) {
        return 0.0f;
    }
    float inter_area  = static_cast<float>(inter.area());
    float union_area  = static_cast<float>(a.area() + b.area()) - inter_area;
    return inter_area / union_area;
}

bool VisionSystem::pointInBox(float px, float py, const cv::Rect& box)
{
    return px >= box.x && px <= (box.x + box.width) &&
           py >= box.y && py <= (box.y + box.height);
}

std::vector<int> VisionSystem::matchDetections(const std::vector<Detection>& dets)
{
    // Age all existing tracks
    for (auto& [id, state] : tracks_) {
        state.age++;
    }

    // Greedy IoU matching: for each detection find best existing track
    std::vector<int> result(dets.size(), -1);
    std::vector<bool> track_used(false);

    // Build a list of track IDs for indexed access
    std::vector<int> track_ids_list;
    for (const auto& [id, _] : tracks_) {
        track_ids_list.push_back(id);
    }
    track_used.resize(track_ids_list.size(), false);

    for (std::size_t di = 0; di < dets.size(); ++di) {
        float best_iou  = IOU_MATCH_THRESHOLD;
        int   best_ti   = -1;

        for (std::size_t ti = 0; ti < track_ids_list.size(); ++ti) {
            if (track_used[ti]) {
                continue;
            }
            int  tid = track_ids_list[ti];
            float iou = computeIoU(dets[di].bbox, tracks_.at(tid).bbox);
            if (iou > best_iou) {
                best_iou = iou;
                best_ti  = static_cast<int>(ti);
            }
        }

        if (best_ti >= 0) {
            // Matched existing track
            int tid            = track_ids_list[best_ti];
            tracks_[tid].bbox  = dets[di].bbox;
            tracks_[tid].age   = 0;
            track_used[best_ti] = true;
            result[di]          = tid;
        } else {
            // New track
            int new_id          = assignNewTrackId();
            tracks_[new_id]     = {dets[di].bbox, 0};
            result[di]          = new_id;
        }
    }

    // Expire old tracks
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        if (it->second.age > MAX_TRACK_AGE) {
            track_to_aruco_.erase(it->first);
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }

    return result;
}

}  // namespace yolo_lidar
