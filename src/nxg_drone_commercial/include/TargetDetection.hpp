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
    typedef struct {
      int white_value_min = 200;
      int white_saturation_max = 40;

      int min_box_width = 30;
      int min_box_height = 30;

      //0 disables max
      int max_box_width = 0;
      int max_box_height = 0;

      int morph_close_size = 15;
      int morph_open_size = 5;

      double min_area = 900;
      double min_aspect = 0.0;
      double max_aspect = 0.0;

      int edge_margin = 2;
      cv::Scalar box_color = cv::Scalar(0, 255, 0);
      int box_thickness = 2;
    } Config;
    cv::VideoWriter writer;
    TargetDetection::Config config;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr detection_results_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub;
};