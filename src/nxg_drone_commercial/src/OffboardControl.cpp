#include "OffboardControl.hpp"

#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "rclcpp/wait_for_message.hpp"

#include "px4_ros_com/frame_transforms.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <functional>
#include <chrono>
#include <cmath>

using namespace std::literals::chrono_literals;

OffboardControl::OffboardControl(OffboardControl::OffboardControlMode mode, uint32_t offboard_control_mode_freq_hz) : Node("offboard_control"), offboard_control_mode(mode), offboard_control_mode_freq_hz(offboard_control_mode_freq_hz) {
    this->offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    this->trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    this->vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    this->visual_inertial_odometry_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

    rclcpp::QoS vehicle_status_qos(10);
    vehicle_status_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    vehicle_status_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    this->vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v2", vehicle_status_qos, [this](px4_msgs::msg::VehicleStatus msg) {
        this->setVehicleStatus(msg);
    });
    rclcpp::QoS vehicle_land_detected_qos(10);
    vehicle_land_detected_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    vehicle_land_detected_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    this->vehicle_land_detected_sub = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", vehicle_land_detected_qos, [this](px4_msgs::msg::VehicleLandDetected msg) {
        this->setVehicleLandStatus(msg);
    });
    this->vio_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odomimu", 10, [this](nav_msgs::msg::Odometry msg) {
        this->publishVIO(msg);
    });
    rclcpp::QoS vehicle_local_position_qos(10);
    vehicle_local_position_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    vehicle_local_position_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    this->vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", vehicle_local_position_qos, [this](px4_msgs::msg::VehicleLocalPosition msg) {
        this->setVehicleLocalPosition(msg);
    });
    this->vehicle_direction_sub = this->create_subscription<std_msgs::msg::Float64>("/offboard_control/direction", 10, [this](std_msgs::msg::Float64 msg) {
        this->radians = msg.data;
        if (this->distance != 0) {
            this->distance_x = std::cos(this->radians) * this->distance;
            this->distance_y = std::sin(this->radians) * this->distance;
            this->ready = true;
        }
    });
    this->vehicle_distance_sub = this->create_subscription<std_msgs::msg::Float64>("/offboard_control/distance", 10, [this](std_msgs::msg::Float64 msg) {
        this->distance = msg.data;
    });
    std::chrono::duration<double> offboard_control_mode_callback_period(1.0 / this->offboard_control_mode_freq_hz);
    this->offboard_control_mode_callback_timer = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(offboard_control_mode_callback_period), std::bind(&OffboardControl::publishOffboardControlMode, this));
    this->offboard_controller = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(3s), std::bind(&OffboardControl::offboardController, this));
}

void OffboardControl::setVehicleLocalPosition(px4_msgs::msg::VehicleLocalPosition msg) {
    if (!msg.xy_valid) {
        std::cout << "XY Position Invalid\n";
        return;
    }
    if (!msg.z_valid) {
        std::cout << "Z Position Invalid\n";
        return;
    }
    this->vehicle_position.x_m = msg.x;
    this->vehicle_position.y_m = msg.y;
    this->vehicle_position.z_m = msg.z;
}

void OffboardControl::offboardController(void) {
    static OffboardControl::FlightState state = OffboardControl::FlightState::ENABLE_OFFBOARD;
    return;
    switch (state) {
        case OffboardControl::FlightState::ENABLE_OFFBOARD:
            if (this->vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                state = OffboardControl::FlightState::ARM;
            } else {
                std::cout << "Enabling Offboard Control\n";
                this->enableOffboardControl();
            }
            break;
        case OffboardControl::FlightState::ARM:
            if (this->ready) {
                std::cout << "Arming\n";
                //this->arm();
                state = OffboardControl::FlightState::TAKEOFF;
            }
            break;
        case OffboardControl::FlightState::TAKEOFF:
            if (this->isArmed()) {
                std::cout << "Taking Off\n";
                this->setTrajectory(std::array<float, 3>{0.0f, 0.0f, -10.0f}, radians);
                state = OffboardControl::FlightState::MOVEMENT;
            }
            break;
        case OffboardControl::FlightState::MOVEMENT:
            if (this->inBounds(this->vehicle_position.z_m, -10.0f, 0.2f)) {
                std::cout << "Moving\n";
                this->setTrajectory(std::array<float, 3>{static_cast<float>(distance_x), static_cast<float>(distance_y), -10.0f}, radians);
                state = OffboardControl::FlightState::LANDING;
            }
            break;
        case OffboardControl::FlightState::LANDING:
            if (this->inBounds(static_cast<double>(this->vehicle_position.x_m), distance_x, 0.5) && this->inBounds(static_cast<double>(this->vehicle_position.y_m), distance_y, 0.5)) {
                std::cout << "Landing\n";
                this->land();
                state = OffboardControl::FlightState::LANDED;
            }
            break;
        case OffboardControl::FlightState::LANDED:
            if (this->isLanded()) {
                std::cout << "Landed\n";
                this->disarm();
                state = OffboardControl::FlightState::FINISHED;
            }
            break;
        case OffboardControl::FlightState::FINISHED:
            std::cout << "Finished\n";
            break;
    }
}

template <typename T>
bool OffboardControl::inBounds(T value, T target, T bound) {
    return (value >= (target - bound)) && (value <= (target + bound));
}

void OffboardControl::setOffboardControlMode(OffboardControl::OffboardControlMode mode) {
    this->offboard_control_mode = mode;
}

bool OffboardControl::enableOffboardControl() {
    if (this->vehicle_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        this->sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    }
    return true;
    // px4_msgs::msg::VehicleStatus status;
    // bool err = rclcpp::wait_for_message(status, this->shared_from_this(), "/fmu/out/vehicle_status_v1");
    // if (!err) {
    //     std::cerr << "Vehicle Status Not Received When Enabling Offboard Control\n";
    //     return false;
    // }
    // return status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

void OffboardControl::land() {
    this->sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

void OffboardControl::arm() {
    this->sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardControl::disarm() {
    this->sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void OffboardControl::takeoff(float altitude_m) {
    this->sendVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m);
}

void OffboardControl::setTrajectory(std::array<float, 3> point, float yaw) {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {point[0], point[1], point[2]};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->trajectory_setpoint_pub->publish(msg);
}

void OffboardControl::sendVehicleCommand(uint16_t command, float param_1, float param_2, float param_3, float param_4, float param_5, float param_6, float param_7) {
    px4_msgs::msg::VehicleCommand msg = {};
    msg.param1 = param_1;
    msg.param2 = param_2;
    msg.param3 = param_3;
    msg.param4 = param_4;
    msg.param5 = param_5;
    msg.param6 = param_6;
    msg.param7 = param_7;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->vehicle_command_pub->publish(msg);
}

//Takes OpenVINS VIO Messages and Translates to PX4 VehicleOdometry
void OffboardControl::publishVIO(nav_msgs::msg::Odometry msg) {
    px4_msgs::msg::VehicleOdometry vio_msg;
    //vio_msg.timestamp = ((uint32_t)(msg.header.stamp.sec) * 1e6) + ((uint32_t)(msg.header.stamp.nsec) / 1000);
    // vio_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // vio_msg.timestamp_sample = vio_msg.timestamp;
    // vio_msg.pose_frame = 1;   //NED
    // vio_msg.q = {std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN()};
    // vio_msg.position = {(float)msg.pose.pose.position.y, (float)msg.pose.pose.position.x, (float)-msg.pose.pose.position.z};
    // vio_msg.velocity_frame = 1;     //NED
    // vio_msg.velocity = {(float)msg.twist.twist.linear.y, (float)msg.twist.twist.linear.x, (float)-msg.twist.twist.linear.z};
    // vio_msg.angular_velocity = {(float)msg.twist.twist.angular.y, (float)msg.twist.twist.angular.x, (float)-msg.twist.twist.angular.z};
    // vio_msg.position_variance = {(float)msg.pose.covariance[7], (float)msg.pose.covariance[0], (float)msg.pose.covariance[14]};
    // vio_msg.velocity_variance = {(float)msg.twist.covariance[7], (float)msg.twist.covariance[0], (float)msg.twist.covariance[14]};
    // std::cout << "Velocity Variance: " << vio_msg.velocity_variance[0] << "," << vio_msg.velocity_variance[1] << "," << vio_msg.velocity_variance[2] << "\n";
    // this->visual_inertial_odometry_pub->publish(vio_msg);
    Eigen::Quaterniond orientation = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Quaterniond ned_orientation = px4_ros_com::frame_transforms::enu_to_ned_orientation(px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(orientation));
    Eigen::Vector3d position = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Vector3d ned_position = px4_ros_com::frame_transforms::enu_to_ned_local_frame(position);
    Eigen::Vector3d linear_velocity = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    Eigen::Vector3d ned_linear_velocity = px4_ros_com::frame_transforms::enu_to_ned_local_frame(linear_velocity);
    Eigen::Vector3d angular_velocity = Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
    Eigen::Vector3d ned_angular_velocity = px4_ros_com::frame_transforms::enu_to_ned_local_frame(angular_velocity);
    vio_msg.q = {ned_orientation.w(), ned_orientation.x(), ned_orientation.y(), ned_orientation.z()};
    vio_msg.position = {ned_position.x(), ned_position.y(), ned_position.z()};
    vio_msg.velocity_frame = 1;     //NED
    vio_msg.velocity = {ned_linear_velocity.x(), ned_linear_velocity.y(), ned_linear_velocity.z()};
    vio_msg.angular_velocity = {ned_angular_velocity.x(), ned_angular_velocity.y(), ned_angular_velocity.z()};
    vio_msg.position_variance = {(float)msg.pose.covariance[7], (float)msg.pose.covariance[0], (float)msg.pose.covariance[14]};
    vio_msg.velocity_variance = {(float)msg.twist.covariance[7], (float)msg.twist.covariance[0], (float)msg.twist.covariance[14]};
    vio_msg.orientation_variance = {(float)msg.pose.covariance[28], (float)msg.pose.covariance[21], (float)msg.pose.covariance[35]};
    vio_msg.timestamp = vio_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    this->visual_inertial_odometry_pub->publish(vio_msg);
}

void OffboardControl::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg = {};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    switch(this->offboard_control_mode) {
        case OffboardControl::OffboardControlMode::POSITION:
            msg.position = true;
            break;
        case OffboardControl::OffboardControlMode::VELOCITY:
            msg.velocity = true;
            break;
        case OffboardControl::OffboardControlMode::ACCELERATION:
            msg.acceleration = true;
            break;
        case OffboardControl::OffboardControlMode::ATTITUDE:
            msg.attitude = true;
            break;
        case OffboardControl::OffboardControlMode::BODY_RATE:
            msg.body_rate = true;
            break;
        case OffboardControl::OffboardControlMode::THRUST_AND_TORQUE:
            msg.thrust_and_torque = true;
            break;
        case OffboardControl::OffboardControlMode::DIRECT_MOTORS_AND_SERVOS:
            msg.direct_actuator = true;
            break;
        default:
            std::cerr << "Selected OffboardControlMode Not Valid\n";
            break;
    }
    this->offboard_control_mode_pub->publish(msg);
}

void OffboardControl::setVehicleStatus(px4_msgs::msg::VehicleStatus status) {
    this->vehicle_status = status;
}

void OffboardControl::setVehicleLandStatus(px4_msgs::msg::VehicleLandDetected status) {
    this->is_landed = status.landed;
}

bool OffboardControl::isArmed() {
    switch (this->vehicle_status.arming_state) {
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED:
            return false;
        case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
            return true;
        default:
            std::cerr << "Arming State is Unknown\n";
            return false;
    }
}

bool OffboardControl::isLanded() {
    return this->is_landed;
}