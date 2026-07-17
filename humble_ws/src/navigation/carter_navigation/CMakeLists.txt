cmake_minimum_required(VERSION 3.5)
project(carter_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  scripts
  params
  maps
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()
