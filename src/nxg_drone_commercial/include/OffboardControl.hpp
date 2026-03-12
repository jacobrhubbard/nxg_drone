#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OffboardControl : public rclcpp::Node {
  public:
    enum class OffboardControlMode {
        POSITION,                   //NED requires position estimate
        VELOCITY,                   //NED requires velocity estimate
        ACCELERATION,               //NED requires velocity estimate
        ATTITUDE,                   //FRD
        BODY_RATE,                  //FRD
        THRUST_AND_TORQUE,          //FRD
        DIRECT_MOTORS_AND_SERVOS,
    };

    enum class FlightStatus {
        LANDED,
        FALLING,
        FLYING
    };

    //NED
    struct {
      float x_m;
      float y_m;
      float z_m;
    } typedef VehiclePosition;

    OffboardControl(OffboardControlMode mode = OffboardControlMode::POSITION, uint32_t offboard_control_mode_hz = 10);

    bool isArmed();
    bool isLanded();
    bool enableOffboardControl();
    

    void land();
    void arm();
    void disarm();
    void takeoff(float altitude_m = 10.0);
    void setTrajectory(std::array<float, 3> point);
    void setOffboardControlMode(OffboardControlMode mode);
  private:
    rclcpp::TimerBase::SharedPtr offboard_control_mode_callback_timer, offboard_controller;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_inertial_odometry_pub;

    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub;

    OffboardControlMode offboard_control_mode;
    VehiclePosition vehicle_position;
    uint32_t offboard_control_mode_freq_hz;

    bool is_landed = true;

    px4_msgs::msg::VehicleStatus vehicle_status;

    void sendVehicleCommand(uint16_t command, float param_1 = 0.0f, float param_2 = 0.0f, float param_3 = 0.0f, float param_4 = 0.0f, float param_5 = 0.0f, float param_6 = 0.0f, float param_7 = 0.0f);
    void publishOffboardControlMode(void);
    void publishVIO(nav_msgs::msg::Odometry msg);
    void setVehicleStatus(px4_msgs::msg::VehicleStatus status);
    void setVehicleLandStatus(px4_msgs::msg::VehicleLandDetected status);
    void setVehicleLocalPosition(px4_msgs::msg::VehicleLocalPosition msg);

    template <typename T>
    bool inBounds(T value, T target, T bound);

    void offboardController(void);
};