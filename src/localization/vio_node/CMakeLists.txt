cmake_minimum_required(VERSION 3.8)
project(vio_node)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc video)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(vio_node src/vio_node.cpp)
target_compile_features(vio_node PUBLIC cxx_std_17)
target_compile_options(vio_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(vio_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vio_node ${OpenCV_LIBRARIES})
ament_target_dependencies(
  vio_node
  cv_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros)

install(TARGETS vio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
