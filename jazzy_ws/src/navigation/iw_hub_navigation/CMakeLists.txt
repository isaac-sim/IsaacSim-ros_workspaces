cmake_minimum_required(VERSION 3.5)
project(iw_hub_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  behavior_trees
  rviz2
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/warehouse_transfer_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME warehouse_transfer_mission)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_warehouse_transfer_mission
    test/test_warehouse_transfer_mission.py)
endif()

ament_package()
