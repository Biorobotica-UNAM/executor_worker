cmake_minimum_required(VERSION 3.5)
project(tasks)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(listener_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(hri_tools REQUIRED)
find_package(hardware_tools REQUIRED)
find_package(navigation_tools REQUIRED)
find_package(service REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${listener_interfaces_INCLUDE_DIRS}
  ${known_locations_interfaces_INCLUDE_DIRS}
  ${hri_tools_INCLUDE_DIRS}
  ${hardware_tools_INCLUDE_DIRS}
  ${navigation_tools_INCLUDE_DIRS}
  ${service_INCLUDE_DIRS}
)

add_executable(robot_inspection src/robot_inspection.cpp)
add_executable(demo src/demo.cpp)
add_executable(help_me_carry src/help_me_carry.cpp)


ament_target_dependencies(robot_inspection
  rclcpp
  std_msgs
  listener_interfaces
  known_locations_interfaces
  hri_tools
  hardware_tools
  navigation_tools
  service
)

ament_target_dependencies(demo
  rclcpp
  std_msgs
  listener_interfaces
  known_locations_interfaces
  hri_tools
  hardware_tools
  navigation_tools
  service
)

ament_target_dependencies(help_me_carry
  rclcpp
  std_msgs
  listener_interfaces
  known_locations_interfaces
  hri_tools
  hardware_tools
  navigation_tools
  service
)

install(TARGETS robot_inspection demo help_me_carry
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
