#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>

class TargetDetection : public rclcpp::Node {
  public:
    TargetDetection(void);
    std::vector<cv::Rect> findWhiteBoxes(const cv::Mat &src, cv::Mat &annotated);
  private:
    bool touchesBorder(const cv::Rect &target, const cv::Size &imgSize, int margin);

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr detection_results_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub;
}