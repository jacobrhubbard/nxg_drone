#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>

class PX4Translation : public rclcpp::Node {
  public:
    PX4Translation(float angle_offset = 0.0f);
  private:
    static constexpr unsigned int NUM_SECTORS = 72;
    float angle_offset;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_laserscan_sub;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr px4_obstacle_distance_pub;
    void translateScan();
};