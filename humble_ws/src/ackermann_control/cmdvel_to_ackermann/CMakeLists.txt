cmake_minimum_required(VERSION 3.5)
project(cmdvel_to_ackermann)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/cmdvel_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
