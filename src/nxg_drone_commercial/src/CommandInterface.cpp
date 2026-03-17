#include <iostream>
#include <string>

#include "CommandInterface.hpp"

CommandInterface::CommandInterface() : Node("command_interface") {
    this->vehicle_direction_pub = this->create_publisher<std_msgs::msg::Float64>("offboard_control/direction", 10);
    this->vehicle_distance_pub = this->create_publisher<std_msgs::msg::Float64>("offboard_control/distance", 10);
}

void CommandInterface::Controller() {
    std::string input << std::cin;
    std::string command = input.substr(0, 4);
    switch (command) {
        case "dist": {
            double distance;
            try {
                distance = std::stod(input.substr(4));
            } catch (const std::invalid_argument &e) {
                std::cerr << "Invalid Argument: " << e.what() << "\n";
            } catch (const std::out_of_range &e) {
                std::cerr << "Out of Range: " << e.what() << "\n";
            }
            std_msgs::msg::Float64 msg;
            msg.data = distance;
            this->vehicle_distance_pub->publish(msg);
            break;
        }
        case "dirc": {
            double angle;
            try {
                angle = std::stod(input.substr(4));
            } catch (const std::invalid_argument &e) {
                std::cerr << "Invalid Argument: " << e.what() << "\n";
            } catch (const std::out_of_range &e) {
                std::cerr << "Out of Range: " << e.what() << "\n";
            }
            std_msgs::msg::Float64 msg;
            msg.data = angle;
            this->vehicle_direction_pub->publish(msg);
            break;
        }
    }
}