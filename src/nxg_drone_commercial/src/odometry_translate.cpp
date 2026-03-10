#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class VehicleOdometryToTwistWithCovarianceStampedNode : public rclcpp::Node {
 private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_stamped_pub;
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odom_msg) {
    geometry_msgs::msg::TwistWithCovarianceStamped odom_msg;
    static uint32_t seq_num = 0;
    //Set Up Header
    odom_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(px4_odom_msg->timestamp) * 1000); //Micro to nano second timestamp
    odom_msg.header.frame_id = "base_link";
    //Set Up twist
    switch (px4_odom_msg->velocity_frame) {
      case 1:
        //NED -> ENU
        //Velocities
        odom_msg.twist.twist.linear.x = static_cast<double>(px4_odom_msg->velocity[1]);
        odom_msg.twist.twist.linear.y = static_cast<double>(px4_odom_msg->velocity[0]);
        odom_msg.twist.twist.linear.z = static_cast<double>(-px4_odom_msg->velocity[2]);
        odom_msg.twist.twist.angular.x = static_cast<double>(px4_odom_msg->angular_velocity[1]);
        odom_msg.twist.twist.angular.y = static_cast<double>(px4_odom_msg->angular_velocity[0]);
        odom_msg.twist.twist.angular.z = static_cast<double>(-px4_odom_msg->angular_velocity[2]);
        //Covariances (Variances along diagonal 6x6 with each axis being XYZRollPitchYaw)
        odom_msg.twist.covariance[0] = static_cast<double>(px4_odom_msg->velocity_variance[1]);
        odom_msg.twist.covariance[7] = static_cast<double>(px4_odom_msg->velocity_variance[0]);
        odom_msg.twist.covariance[14] = static_cast<double>(px4_odom_msg->velocity_variance[2]);
        odom_msg.twist.covariance[21] = static_cast<double>(px4_odom_msg->orientation_variance[1]);
        odom_msg.twist.covariance[28] = static_cast<double>(px4_odom_msg->orientation_variance[0]);
        odom_msg.twist.covariance[35] = static_cast<double>(px4_odom_msg->orientation_variance[2]);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "PX4 VehicleOdometry.velocity_frame set to %d which is unsupported", px4_odom_msg->velocity_frame);
        return;
    }
    twist_with_covariance_stamped_pub->publish(odom_msg);
  }
 public:
  VehicleOdometryToTwistWithCovarianceStampedNode() : Node("vehicle_odometry_to_twist_with_covariance_stamped") {
    rclcpp::QoS qos(10);
    qos.best_effort();
    this->vehicle_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&VehicleOdometryToTwistWithCovarianceStampedNode::odometry_callback, this, std::placeholders::_1)
    );
    this->twist_with_covariance_stamped_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle_odometry", 10);
    RCLCPP_INFO(this->get_logger(), "Odometry Translate Node Started");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleOdometryToTwistWithCovarianceStampedNode>());
  rclcpp::shutdown();
  return 0;
}
