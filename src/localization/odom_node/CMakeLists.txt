cmake_minimum_required(VERSION 3.8)
project(odom_node)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom_node src/odom_node.cpp)
target_compile_features(odom_node PUBLIC cxx_std_17)
target_compile_options(odom_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  odom_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros)

install(TARGETS odom_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
