#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

#include <cv_bridge/cv_bridge.hpp>

#include "TargetDetection.hpp"

TargetDetection::TargetDetection(void) : Node("target_detection") {
    this->detection_results_pub = this->create_publisher<geometry_msgs::msg::Point>("/target_detection/point", 10);
    this->color_image_sub = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10, [this](sensor_msgs::msg::Image msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        cv::Mat annotated_image;
        std::vector<cv::Rect> detection_results = this->findWhiteBoxes(image, annotated_image);
        this->detection_results_publisher->publish(detection_results);
    });
}

std::vector<cv::Rect> TargetDetection::findWhiteBoxes(const cv::Mat &src, cv::Mat &annotated) {
    cv::Math hsv;
    cv::cvtColor(src, hsv, cv::COLOR::RGB2HSV);
    cv::Mat whiteMask;
    cv::inRange(hsv, cv::Scalar(0, 0, this->config.white_value_min), cv::Scalar(180, this->config.white_saturation_max, 255), whiteMask);
    cv::Mat close_content = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->config.morph_close_size, this->config.morph_close_size));
    cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, close_content);
    cv::Mat open_content = cv::getStructuringElement(cv::MOPRH_RECT, cv::Size(this->config.morph_open_size, this->config.morph_open_size));
    cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_OPEN, open_content);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> boxes;
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < this->config.min_area) continue;
        cv::Rect r = cv::boundingRect(cnt);
        if (r.width < this->config.min_box_width) continue;
        if (r.height < this->config.min_box_height) continue;
        if (this->config.max_box_width > 0 && r.width > this->config.max_box_width) continue;
        if (this->config.max_box_height >0 && r.height > this->config.max_box_height) continue;
        if (this->config.min_aspect > 0.0 || this->config.max_aspect > 0.0) {
            double aspect = static_cast<double>(r.width) / r.height;
            if (this->config.min_aspect > 0.0 && aspect < this->config.min_aspect) continue;
            if (this->config.max_aspect > 0.0 && aspect > this->config.max_aspect) continue;
        }
        if (this->touchesBorder(r, src.size(), this->config.edge_margin)) continue;
        boxes.push_back(r);
    }

    std::sort(boxes.begin(), boxes.end(), [](const cv::Rect &a, const cv::Rect &b) {
        return (a.y != b.y) ? a.y < b.y : a.x < b.x;
    });
    annotated = src.clone();
    for (size_t i = 0; i < boxes.size(); ++i) {
        cv::rectangle(annotated, boxes[i], this->config.box_color, this->config.box_thickness);
        std::string label = "Box " + std::to_string(i + 1);
        cv::putText(annotated, label, cv::Point(boxes[i].x, std::max(boxes[i].y - 6, 0)), cv::FONT_HERSHEY_SIMPLEX, 0.6, this->config.box_color, 2);
    }
    return boxes;
}

bool TargetDetection::touchesBorder(const cv::Rect &target, const cv::Size &imgSize, int margin) {
    return target.x <= margin || target.y <= margin || (target.x + target.width) >= imgSize.width - margin || (target.y + target.height) >= imgSize.height - margin;
}

// Configuration parameters - tweak these for your use case
struct Config {
    // White detection thresholds (HSV color space)
    int white_value_min     = 200;   // Min brightness (0-255) to be considered "white"
    int white_sat_max       = 40;    // Max saturation to be considered "white"

    // Minimum box dimensions in pixels
    int min_box_width       = 30;
    int min_box_height      = 30;

    // Maximum box dimensions (set to 0 to disable upper bound)
    int max_box_width       = 0;
    int max_box_height      = 0;

    // Morphological kernel sizes for noise cleanup
    int morph_close_size    = 15;    // Fills gaps inside white regions
    int morph_open_size     = 5;     // Removes tiny white specks

    // Contour area filter
    double min_area         = 900;   // px^2  (30x30 minimum)

    // Aspect ratio filter (width/height). 0 = no filter
    double min_aspect       = 0.0;
    double max_aspect       = 0.0;

    // Border threshold: ignore regions touching image edge within N pixels
    int edge_margin         = 2;

    // Visual output
    cv::Scalar box_color    = cv::Scalar(0, 255, 0);  // Green bounding boxes
    int box_thickness       = 2;
};

// ─────────────────────────────────────────────────────────────────────────────
// Returns true if the bounding rect touches the image border
// ─────────────────────────────────────────────────────────────────────────────
bool touchesBorder(const cv::Rect& r, const cv::Size& imgSize, int margin) {
    return r.x <= margin ||
           r.y <= margin ||
           (r.x + r.width)  >= imgSize.width  - margin ||
           (r.y + r.height) >= imgSize.height - margin;
}

// // ─────────────────────────────────────────────────────────────────────────────
// // Detects white boxes in `src` and returns their bounding rectangles.
// // Draws annotations on `vis` (a copy of src).
// // ─────────────────────────────────────────────────────────────────────────────
// std::vector<cv::Rect> findWhiteBoxes(const cv::Mat& src, cv::Mat& vis,
//                                      const Config& cfg)
// {
//     // ── 1. Convert to HSV and threshold for white ──────────────────────────
//     cv::Mat hsv;
//     cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

//     // White = high Value, low Saturation
//     cv::Mat whiteMask;
//     cv::inRange(hsv,
//                 cv::Scalar(0,   0,   cfg.white_value_min),
//                 cv::Scalar(180, cfg.white_sat_max, 255),
//                 whiteMask);

//     // ── 2. Morphological cleanup ───────────────────────────────────────────
//     // Close: fills small holes inside white regions (handles text/image content)
//     cv::Mat closeKernel = cv::getStructuringElement(
//         cv::MORPH_RECT,
//         cv::Size(cfg.morph_close_size, cfg.morph_close_size));
//     cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, closeKernel);

//     // Open: removes small isolated white speckles
//     cv::Mat openKernel = cv::getStructuringElement(
//         cv::MORPH_RECT,
//         cv::Size(cfg.morph_open_size, cfg.morph_open_size));
//     cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_OPEN, openKernel);

//     // ── 3. Find external contours ──────────────────────────────────────────
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(whiteMask, contours, cv::RETR_EXTERNAL,
//                      cv::CHAIN_APPROX_SIMPLE);

//     // ── 4. Filter contours and collect bounding rects ──────────────────────
//     std::vector<cv::Rect> boxes;
//     for (const auto& cnt : contours) {
//         double area = cv::contourArea(cnt);
//         if (area < cfg.min_area) continue;

//         cv::Rect r = cv::boundingRect(cnt);

//         // Size filters
//         if (r.width  < cfg.min_box_width)  continue;
//         if (r.height < cfg.min_box_height) continue;
//         if (cfg.max_box_width  > 0 && r.width  > cfg.max_box_width)  continue;
//         if (cfg.max_box_height > 0 && r.height > cfg.max_box_height) continue;

//         // Aspect ratio filter
//         if (cfg.min_aspect > 0.0 || cfg.max_aspect > 0.0) {
//             double aspect = static_cast<double>(r.width) / r.height;
//             if (cfg.min_aspect > 0.0 && aspect < cfg.min_aspect) continue;
//             if (cfg.max_aspect > 0.0 && aspect > cfg.max_aspect) continue;
//         }

//         // Skip regions that are just the image background
//         if (touchesBorder(r, src.size(), cfg.edge_margin)) continue;

//         boxes.push_back(r);
//     }

//     // ── 5. Sort top-to-bottom, left-to-right ──────────────────────────────
//     std::sort(boxes.begin(), boxes.end(),
//               [](const cv::Rect& a, const cv::Rect& b) {
//                   return (a.y != b.y) ? a.y < b.y : a.x < b.x;
//               });

//     // ── 6. Draw results on visualisation image ─────────────────────────────
//     vis = src.clone();
//     for (size_t i = 0; i < boxes.size(); ++i) {
//         cv::rectangle(vis, boxes[i], cfg.box_color, cfg.box_thickness);

//         // Label
//         std::string label = "Box " + std::to_string(i + 1);
//         cv::putText(vis, label,
//                     cv::Point(boxes[i].x, std::max(boxes[i].y - 6, 0)),
//                     cv::FONT_HERSHEY_SIMPLEX, 0.6, cfg.box_color, 2);
//     }

//     return boxes;
// }

// // ─────────────────────────────────────────────────────────────────────────────
// // main
// // ─────────────────────────────────────────────────────────────────────────────
// int main(int argc, char* argv[])
// {
//     if (argc < 2) {
//         std::cerr << "Usage: " << argv[0]
//                   << " <image_path> [output_path]\n"
//                   << "  output_path defaults to 'result.png'\n";
//         return 1;
//     }

//     const std::string inputPath  = argv[1];
//     const std::string outputPath = (argc >= 3) ? argv[2] : "result.png";

//     // ── Load image ─────────────────────────────────────────────────────────
//     cv::Mat src = cv::imread(inputPath);
//     if (src.empty()) {
//         std::cerr << "Error: could not load image: " << inputPath << "\n";
//         return 1;
//     }
//     std::cout << "Image loaded: " << src.cols << "x" << src.rows
//               << " (" << inputPath << ")\n";

//     // ── Configure ──────────────────────────────────────────────────────────
//     Config cfg;
//     // Uncomment / adjust these to tune detection:
//     // cfg.white_value_min  = 210;
//     // cfg.white_sat_max    = 30;
//     // cfg.morph_close_size = 20;
//     // cfg.min_box_width    = 50;
//     // cfg.min_box_height   = 50;

//     // ── Detect ────────────────────────────────────────────────────────────
//     cv::Mat vis;
//     std::vector<cv::Rect> boxes = findWhiteBoxes(src, vis, cfg);

//     // ── Print results ──────────────────────────────────────────────────────
//     std::cout << "Found " << boxes.size() << " white box(es):\n";
//     for (size_t i = 0; i < boxes.size(); ++i) {
//         const cv::Rect& r = boxes[i];
//         std::cout << "  Box " << (i + 1) << ": "
//                   << "x=" << r.x << " y=" << r.y
//                   << " w=" << r.width << " h=" << r.height
//                   << " (area=" << r.area() << " px^2)\n";
//     }

//     // ── Save output ───────────────────────────────────────────────────────
//     cv::imwrite(outputPath, vis);
//     std::cout << "Result saved to: " << outputPath << "\n";

//     // Also save the binary mask for debugging
//     cv::Mat hsv, whiteMask;
//     cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
//     cv::inRange(hsv,
//                 cv::Scalar(0, 0, cfg.white_value_min),
//                 cv::Scalar(180, cfg.white_sat_max, 255),
//                 whiteMask);
//     cv::imwrite("debug_mask.png", whiteMask);
//     std::cout << "Debug mask saved to: debug_mask.png\n";

//     return 0;
// }
