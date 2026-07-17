cmake_minimum_required(VERSION 3.16)
project(isaac_ros2_control_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz srdf config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/read_robot_description.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME read_robot_description)

ament_package()
