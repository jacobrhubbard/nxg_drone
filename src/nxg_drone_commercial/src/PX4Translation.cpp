#include "PX4Translation.hpp"

PX4Translation::PX4Translation(float angle_offset) : Node("px4_translation"), angle_offset(angle_offset) {
    this->px4_obstacle_distance_pub = this->create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
    this->lidar_laserscan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/rplidar_a2m12/scan", rclcpp::QoS(rclcpp::KeepLast(10)), [this](sensor_msgs::msg::LaserScan msg) {
        px4_msgs::msg::ObstacleDistance pub_msg;
        pub_msg.frame = px4_msgs::msg::ObstacleDistance::MAV_FRAME_BODY_FRD;
        pub_msg.sensor_type = px4_msgs::msg::ObstacleDistance::MAV_DISTANCE_SENSOR_LASER;
        pub_msg.min_distance = static_cast<float>(msg.range_min * 100.0f);
        pub_msg.max_distance = static_cast<float>(msg.range_max * 100.0f);
        pub_msg.increment = msg.angle_increment * 180.0f / M_PI;
        pub_msg.timestamp = (msg.header.stamp.sec * 1000000) + (msg.header.stamp.nanosec / 1000);
        //PX4 Internal Obstacle Distance Map has 72 Sectors
        //Therefore we create 72 bins and find the minimum value (closest to the drone) and publish that as the read value to the sector.
        unsigned int total_scanner_points = msg.ranges.size();
        unsigned int num_points_per_sector = (total_scanner_points + PX4Translation::NUM_SECTORS - 1) / PX4Translation::NUM_SECTORS;
        for (int sector = 0; sector < PX4Translation::NUM_SECTORS; ++sector) {
            float min_value_meters = std::numeric_limits<float>::max();
            for (int i = 0; i < num_points_per_sector; ++i) {
                float val = msg.ranges[i + (num_points_per_sector * sector)];
                if (val < min_value_meters) {
                    if (val > msg.range_min && val < msg.range_max) {
                        min_value_meters = val;
                    }
                }
            }
            pub_msg.distances[sector] = min_value_meters == std::numeric_limits<float>::max() ? pub_msg.max_distance + 1 : static_cast<uint16_t>(min_value_meters * 100.0f);
        }
        this->px4_obstacle_distance_pub->publish(pub_msg);
    });
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Translation>());
  rclcpp::shutdown();
  return 0;
}