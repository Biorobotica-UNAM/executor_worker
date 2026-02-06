cmake_minimum_required(VERSION 3.5)
project(tasks)

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
find_package(manipulation_planner REQUIRED)

include_directories(
  include
)

add_executable(robot_inspection src/robot_inspection.cpp)
add_executable(demo src/demo.cpp)
add_executable(help_me_carry src/help_me_carry.cpp)
add_executable(panHead src/panHead.cpp)
add_executable(tasks_node src/tasks_node.cpp)

ament_target_dependencies(robot_inspection
  rclcpp
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
  manipulation_planner
)

ament_target_dependencies(demo
  rclcpp
  std_msgs
  listener_interfaces
  talker_interfaces
  known_locations_interfaces
  clips_interfaces
  manipulation_planner_interfaces
  hri_tools
  hardware_tools
  manipulation_tools
  navigation_tools
  planning_tools
  manipulation_planner
)

ament_target_dependencies(help_me_carry
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  tf2_ros
  listener_interfaces
  talker_interfaces
  known_locations_interfaces
  clips_interfaces
  manipulation_planner_interfaces
  hri_tools
  hardware_tools
  manipulation_tools
  navigation_tools
  planning_tools
  manipulation_planner
)

ament_target_dependencies(panHead
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  tf2_ros
  listener_interfaces
  talker_interfaces
  known_locations_interfaces
  hri_tools
  hardware_tools
)

ament_target_dependencies(tasks_node
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
  manipulation_planner
)

install(TARGETS
  robot_inspection
  demo
  help_me_carry
  panHead
  tasks_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
