#include <cstdio>

#include <thread>
#include <chrono>
#include <iostream>

#include "OffboardControl.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>(OffboardControl::OffboardControlMode::POSITION, 10));
  rclcpp::shutdown();
  return 0;
}
