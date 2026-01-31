cmake_minimum_required(VERSION 3.8)
project(driver_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rover_messages REQUIRED)

# Add C++ executable
add_executable(goal_pose_to_drive src/goal_pose_to_drive.cpp)

ament_target_dependencies(goal_pose_to_drive
  rclcpp
  geometry_msgs
  std_msgs
  rover_messages
)


# Install the executable
install(TARGETS goal_pose_to_drive
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
