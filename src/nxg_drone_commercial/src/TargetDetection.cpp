#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

#include <cv_bridge/cv_bridge.hpp>

#include "TargetDetection.hpp"

TargetDetection::TargetDetection(void) : Node("target_detection") {
    this->detection_results_pub = this->create_publisher<geometry_msgs::msg::Point>("/target_detection/point", 10);
    this->annotated_image_pub = this->create_publisher<sensor_msgs::msg::Image>("target_detection/annotated", 10);
    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    this->writer = cv::VideoWriter("output.mp4", codec, 15.0, cv::Size(640,480));
    if (!writer.isOpened()) {
        std::cerr << "Error video writer\n";
        return;
    }
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
        geometry_msgs::msg::Point point_result;
        for (cv::Rect result : detection_results) {
            geometry_msgs::msg::Point point;
            point.x = static_cast<double>(result.x) + (static_cast<double>(result.width) / 2.0);
            point.y = static_cast<double>(result.y) + (static_cast<double>(result.height) / 2.0);
            this->detection_results_pub->publish(point);
        }
        writer.write(annotated_image);
    });
}

std::vector<cv::Rect> TargetDetection::findWhiteBoxes(const cv::Mat &src, cv::Mat &annotated) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_RGB2HSV);
    cv::Mat whiteMask;
    cv::inRange(hsv, cv::Scalar(0, 0, this->config.white_value_min), cv::Scalar(180, this->config.white_saturation_max, 255), whiteMask);
    cv::Mat close_content = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->config.morph_close_size, this->config.morph_close_size));
    cv::morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, close_content);
    cv::Mat open_content = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->config.morph_open_size, this->config.morph_open_size));
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

int main(int argc, char ** argv) {
    (void) argc;
    (void) argv;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetDetection>());
    rclcpp::shutdown();
    return 0;
}