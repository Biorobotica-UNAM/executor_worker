cmake_minimum_required(VERSION 3.5)
project(executor_worker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(listener_interfaces REQUIRED)
find_package(executor_interfaces REQUIRED)
find_package(talker_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)
find_package(clips_interfaces REQUIRED)

find_package(hri_tools REQUIRED)
find_package(hardware_tools REQUIRED)
find_package(manipulation_tools REQUIRED)
find_package(navigation_tools REQUIRED)
find_package(planning_tools REQUIRED)
find_package(executor_tools REQUIRED)
find_package(manipulation_planner REQUIRED)

include_directories(
  include
)

add_executable(executor_worker_node src/executor_worker_node.cpp)

ament_target_dependencies(executor_worker_node
  rclcpp
  rclcpp_action
  std_msgs
  listener_interfaces
  talker_interfaces
  known_locations_interfaces
  executor_interfaces
  clips_interfaces
  manipulation_planner_interfaces
  hri_tools
  hardware_tools
  manipulation_tools
  navigation_tools
  planning_tools
  executor_tools
  manipulation_planner
)

install(TARGETS
  executor_worker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
