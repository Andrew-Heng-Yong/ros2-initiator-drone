cmake_minimum_required(VERSION 3.8)
project(mpu6050_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mpu6050_node src/mpu6050_node.cpp)
target_compile_features(mpu6050_node PUBLIC cxx_std_17)
ament_target_dependencies(mpu6050_node rclcpp sensor_msgs)

install(TARGETS mpu6050_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
