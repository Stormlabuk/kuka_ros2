cmake_minimum_required(VERSION 3.5)
project(robot_comms_velocity)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kuka_messages REQUIRED)

# Include directories (if you have any additional include directories)
include_directories(include)

# Add executables
add_executable(transmit_robot_node src/transmit_robot.cpp)
add_executable(receive_robot_node src/receive_robot.cpp)
add_executable(heartbeat_node src/heartbeat.cpp)  # Added heartbeat node

# Link libraries
ament_target_dependencies(transmit_robot_node
  rclcpp
  sensor_msgs
  std_msgs
  kuka_messages
)
ament_target_dependencies(receive_robot_node
  rclcpp
  sensor_msgs
  std_msgs
  kuka_messages
)
ament_target_dependencies(heartbeat_node  # Added dependencies for heartbeat node
  rclcpp
)

# Install targets
install(TARGETS
  transmit_robot_node
  receive_robot_node
  heartbeat_node        # Install heartbeat node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

