#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class CommandInterface : public rclcpp::Node {
  public:
    CommandInterface();
  private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vehicle_direction_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vehicle_distance_pub;

    void Controller();
}