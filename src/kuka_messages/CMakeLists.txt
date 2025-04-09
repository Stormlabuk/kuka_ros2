cmake_minimum_required(VERSION 3.5)
project(kuka_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ControlMessage.msg"
  "msg/MovingStatus.msg"
  "msg/CartesianPose.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces  # Add this line
)

ament_package()


