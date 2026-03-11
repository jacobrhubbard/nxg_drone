cmake_minimum_required(VERSION 3.8)
project(nxg_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(px4_ros_com REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(TargetDetection src/TargetDetection.cpp)
ament_target_dependencies(TargetDetection rclcpp px4_msgs OpenCV cv_bridge)
add_executable(commercial src/commercial.cpp src/OffboardControl.cpp)
ament_target_dependencies(commercial rclcpp px4_msgs nav_msgs px4_ros_com)
add_executable(odometry_translate src/odometry_translate.cpp)
ament_target_dependencies(odometry_translate rclcpp geometry_msgs nav_msgs px4_msgs)

target_include_directories(TargetDetection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(commercial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(commercial PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

  install(TARGETS TargetDetection
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS odometry_translate
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS commercial
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS main
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
