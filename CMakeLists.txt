cmake_minimum_required(VERSION 3.8)
project(executor_worker)

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(executor_interfaces REQUIRED)

find_package(executor_tools REQUIRED)

# -----------------------------
# Executable
# -----------------------------
add_executable(executor_worker_node src/executor_worker_node.cpp)

ament_target_dependencies(executor_worker_node
  rclcpp
  rclcpp_action
  executor_interfaces
  executor_tools
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  executor_worker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
