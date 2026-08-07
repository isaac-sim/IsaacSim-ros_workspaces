cmake_minimum_required(VERSION 3.16)
project(isaac_ros2_control_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch rviz srdf config DESTINATION share/${PROJECT_NAME})
ament_python_install_package(
  ${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

ament_package()
