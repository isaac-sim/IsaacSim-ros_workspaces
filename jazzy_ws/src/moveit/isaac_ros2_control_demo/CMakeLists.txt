cmake_minimum_required(VERSION 3.16)
project(isaac_ros2_control_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz srdf config urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/benchmark_trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME benchmark_trajectory_publisher)
install(PROGRAMS scripts/ur10_motion_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ur10_motion_test)

ament_package()
