cmake_minimum_required(VERSION 3.5)
project(isaac_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit REQUIRED)

install(DIRECTORY
  rviz2
  launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/gripper_to_isaac.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
