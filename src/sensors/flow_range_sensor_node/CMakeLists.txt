cmake_minimum_required(VERSION 3.8)
project(flow_range_sensor_node)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OpticalFlow.msg"
  DEPENDENCIES std_msgs
)

add_library(flow_range_drivers
  src/pmw3901.cpp
  src/two_wire_linux.cpp
  third_party/vl53l1x/VL53L1X.cpp
)
target_compile_features(flow_range_drivers PUBLIC cxx_std_17)
target_include_directories(flow_range_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l1x>
  $<INSTALL_INTERFACE:include>
)

add_executable(flow_range_sensor_node src/flow_range_sensor_node.cpp)
target_compile_features(flow_range_sensor_node PUBLIC cxx_std_17)
target_compile_options(flow_range_sensor_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(flow_range_sensor_node flow_range_drivers)
ament_target_dependencies(flow_range_sensor_node diagnostic_msgs rclcpp sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(flow_range_sensor_node "${cpp_typesupport_target}")

install(TARGETS
  flow_range_drivers
  flow_range_sensor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES third_party/vl53l1x/LICENSE.txt
  DESTINATION share/${PROJECT_NAME}/third_party/vl53l1x)
install(FILES third_party/pmw3901/LICENSE.txt
  DESTINATION share/${PROJECT_NAME}/third_party/pmw3901)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
