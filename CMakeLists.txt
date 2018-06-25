cmake_minimum_required(VERSION 3.5)
project(adlink_ros2_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(adlink_msgs REQUIRED)

# RoadTrip_ping
add_executable(RoundTrip_ping src/RoundTrip_ping.cpp)
ament_target_dependencies(RoundTrip_ping rclcpp std_msgs adlink_msgs)

# RoadTrip_ping
add_executable(RoundTrip_pong src/RoundTrip_pong.cpp)
ament_target_dependencies(RoundTrip_pong rclcpp std_msgs adlink_msgs)

install(TARGETS
  RoundTrip_ping
  RoundTrip_pong
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
