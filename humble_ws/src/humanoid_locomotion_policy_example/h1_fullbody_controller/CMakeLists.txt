cmake_minimum_required(VERSION 3.5)
project(h1_fullbody_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY
  scripts
  launch
  policy
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/h1_fullbody_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
