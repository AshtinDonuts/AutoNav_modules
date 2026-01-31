cmake_minimum_required(VERSION 3.10)
project(rover_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Needed if any message uses std_msgs types

# Message files
set(msg_files
  "msg/Drive.msg"
  "msg/DriveVelocity.msg"
  "msg/DriveSteer.msg"
  "msg/DriveStatus.msg"
)

# Service files (if you have any, otherwise empty)
set(srv_files
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_package()
