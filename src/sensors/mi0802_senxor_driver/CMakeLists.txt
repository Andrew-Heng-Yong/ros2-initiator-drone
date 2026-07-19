cmake_minimum_required(VERSION 3.8)
project(mi0802_senxor_driver)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(mi0802_protocol src/mi0802_device.cpp)
target_compile_features(mi0802_protocol PUBLIC cxx_std_17)
target_include_directories(mi0802_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(mi0802_protocol PRIVATE -Wall -Wextra -Wpedantic)

add_library(mi0802_frame_utils src/frame_utils.cpp)
target_compile_features(mi0802_frame_utils PUBLIC cxx_std_17)
target_include_directories(mi0802_frame_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mi0802_frame_utils PUBLIC mi0802_protocol Threads::Threads)
ament_target_dependencies(mi0802_frame_utils builtin_interfaces sensor_msgs)
target_compile_options(mi0802_frame_utils PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mi0802_senxor_node src/mi0802_senxor_node.cpp)
target_compile_features(mi0802_senxor_node PUBLIC cxx_std_17)
target_link_libraries(mi0802_senxor_node mi0802_protocol mi0802_frame_utils)
ament_target_dependencies(mi0802_senxor_node rclcpp sensor_msgs)
target_compile_options(mi0802_senxor_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS mi0802_protocol mi0802_frame_utils mi0802_senxor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mi0802_driver test/test_mi0802_driver.cpp)
  target_link_libraries(test_mi0802_driver mi0802_protocol mi0802_frame_utils)
  ament_target_dependencies(test_mi0802_driver builtin_interfaces sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(builtin_interfaces sensor_msgs)
ament_package()
