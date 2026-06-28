cmake_minimum_required(VERSION 3.8)
project(mlx90640_node)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(mlx90640_reference STATIC
  drivers/mlx90640/MLX90640_API.c
  src/mlx90640_i2c_linux.c)
target_include_directories(mlx90640_reference PUBLIC include drivers/mlx90640)
set_target_properties(mlx90640_reference PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED YES)
target_link_libraries(mlx90640_reference m)

add_executable(mlx90640_node src/mlx90640_node.cpp)
target_compile_features(mlx90640_node PUBLIC cxx_std_17)
target_link_libraries(mlx90640_node mlx90640_reference)
ament_target_dependencies(mlx90640_node rclcpp sensor_msgs)

add_executable(thermal_overlay_node src/thermal_overlay_node.cpp)
target_compile_features(thermal_overlay_node PUBLIC cxx_std_17)
target_include_directories(thermal_overlay_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(thermal_overlay_node ${OpenCV_LIBRARIES})
ament_target_dependencies(thermal_overlay_node cv_bridge rcl_interfaces rclcpp sensor_msgs)

install(TARGETS mlx90640_node thermal_overlay_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
