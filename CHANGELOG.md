# Changelog

## [2.0.0] - 2024-09-20

### Added
- New `iw_hub_navigation` package to run Nav2 with new iw.hub robot in new environment [Foxy, Humble]
- New RViz config for TurtleBot tutorials [Noetic]
- New RViz config and launch file with Carter robot for SLAM with gmapping

### Changed
- Bumped versions in `isaacsim` package to Isaac Sim version 4.2.0 [Foxy, Humble]
- Changed `carter_2dnav` package to only use RTX Lidar [Noetic]
- Updated dockerfiles to use setuptools 70.0.0 [Humble, Foxy]
- Updated QoS settings for image subscribers in ``carter_stereo.rviz`` and ``carter_navigation.rviz`` config files. [Foxy, Humble] 

## [1.1.0] - 2024-08-01

### Added
- Option in `isaacsim` package to launch Isaac Sim in headless mode [Foxy, Humble]

### Changed
- Bumped versions in `isaacsim` package to Isaac Sim version 4.1.0 [Foxy, Humble]


## [1.0.0] - 2024-05-28

### Added
- Ackermann publisher script for Ackermann Steering tutorial [Noetic, Foxy, Humble]
- New `isaacsim` package to enable running Isaac Sim as a ROS2 node or from a ROS2 launch file! [Foxy, Humble]
- `isaac_ros2_messages` service interfaces for listing prims and manipulate their attributes [Foxy, Humble]

### Removed
- Removed support for quadruped VINS Fusion example [Noetic]

## [0.1.0] - 2023-12-18
### Added
- Noetic, Foxy, Humble workspaces for Isaac Sim 2023.1.1
