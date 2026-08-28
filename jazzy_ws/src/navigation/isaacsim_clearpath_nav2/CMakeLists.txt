cmake_minimum_required(VERSION 3.8)
project(isaacsim_clearpath_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME})

ament_python_install_package(
  ${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

ament_package()
