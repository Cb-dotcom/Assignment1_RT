cmake_minimum_required(VERSION 3.8)
project(assignment1_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(
  include
)

add_executable(distance_monitor_node
  src/distance_monitor_node.cpp
)

add_executable(turtle_ui_node
  src/turtle_ui_node.cpp
)

ament_target_dependencies(distance_monitor_node
  rclcpp
  geometry_msgs
  std_msgs
  turtlesim
)

ament_target_dependencies(turtle_ui_node
  rclcpp
  geometry_msgs
)

install(TARGETS 
  distance_monitor_node
  turtle_ui_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
