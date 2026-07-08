cmake_minimum_required(VERSION 3.8)
project(vrpn_px4_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(vrpn_px4_bridge_node src/vrpn_px4_bridge_node.cpp)
target_include_directories(vrpn_px4_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vrpn_px4_bridge_node rclcpp geometry_msgs px4_msgs)

install(TARGETS vrpn_px4_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Code style is enforced by the project's own clang-format config, not the
  # ROS ament linters (they conflict). Only the behavioural gtest runs here.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
  target_include_directories(test_frame_transforms PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
