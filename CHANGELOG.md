# Changelog

## [4.5.1] - 2025-10-07
### Changed
- Updated MoveIt configs to mitigate timeout issues [Jazzy]

## [4.5.0] - 2025-09-29
### Changed
- Updated links and bumped versions in all packages to Isaac Sim version 5.1.0 [Humble, Jazzy]

## [4.4.4] - 2025-09-29
### Changed
- Updated setgoal.py in `isaac_ros_navigation_goal` for NavigateToPose action result handling [Humble, Jazzy]

## [4.4.3] - 2025-08-14
### Changed
- `.gitignore` to ignore `.vscode/`.

## [4.4.2] - 2025-08-06

### Changed
- Upgraded configs in `carter_navigation` and `iw_hub_navigation` packages for Nav2 Jazzy [Jazzy]

## [4.4.1] - 2025-07-23

### Changed
- Updated links within docker build files [Humble, Jazzy]
- Updated licenses for all scripts [Humble, Jazzy]

## [4.4.0] - 2025-07-16

### Added
- Added `exclude_install_path` arguments in `isaacsim` package to allow removing certain install paths from environment variables when launching Isaac Sim [Humble, Jazzy]

### Changed
- The `use_internal_libs` parameter in `isaacsim` package is set to true by default [Humble, Jazzy]

## [4.3.1] - 2025-06-11

### Changed
- Bumped versions in `isaacsim` package to Isaac Sim version 5.0.0 [Humble, Jazzy]
- Updated all asset paths referenced in ROS 2 packages to Isaac Sim 5.0 convention

## [4.3.0] - 2025-06-04

### Added
- Humanoid locomotion policy example [Humble, Jazzy]

## [4.2.1] - 2025-06-03

### Changed
- Updating the H1 joint names in the wholebody controller package [Humble]

## [4.2.0] - 2025-05-30

### Added
- Launch file in carter_navigation for running the nova carter robot description [Humble, Jazzy]

## [4.1.0] - 2025-05-20

### Added
- Build scripts for Humble and Jazzy workspaces in Python 3.11 [Humble, Jazzy]

## [4.0.0] - 2025-04-30

### Added
- New Jazzy workspace for Isaac Sim 5.0 [Jazzy]
- New Moveit tutorial [Jazzy, Humble] 

### Changed
- Bumped verison to Isaac Sim 5.0 [Humble]

### Removed
- Removed all Noetic packages and dockerfiles [Noetic]

## [3.3.1] - 2025-01-24

### Changed
- Updated fixed frame name to base_scan in `isaac_tutorials` rtx_lidar.rviz config file [Humble]
- Added point cloud topic subscriber in `isaac_tutorials` camera_lidar.rviz config file [Humble]

## [3.3.0] - 2025-01-21

### Changed
- Updated `isaacsim` run_isaacsim.py to use new commands for "webrtc" headless option [Humble]
- Removed deprecated "native" headless option from `isaacsim` run_isaacsim.py [Humble]

## [3.2.3] - 2025-01-21

### Changed
- Changed `isaacsim` run_isaacsim.py to have "humble" as default distro for ROS2 [Humble]

## [3.2.2] - 2024-01-20

### Changed
- Updated ros2_ackeramnn_publisher.py to publish zero-velocity on interrupt [Humble]

## [3.2.1] - 2024-01-07

### Changed
- Changed `isaac_tutorials` ros2 ackermann publisher message frame id to "ackeramnn" [Humble]

## [3.2.0] - 2024-12-16

### Changed
- Bumped versions in `isaacsim` package to Isaac Sim version 4.5.0 [Humble]
- Updated all asset paths referenced in ROS 2 packages to Isaac Sim 4.5 convention

## [3.1.1] - 2024-11-27

### Changed
- Updated ros2_ackeramnn_publisher.py to publish velocity command [Humble]

## [3.1.0] - 2024-11-22

### Added
- `cmdvel_to_ackermann` package for ackeramnn control [Humble]

## [3.0.1] - 2024-11-22

### Fixed
- updated ubuntu_20_humble_minimal.dockerfile to build all packages [Humble]

## [3.0.0] - 2024-11-15

### Removed
- Removed all Foxy packages and dockerfiles [Foxy]

## [2.1.0] - 2024-10-03

### Added
- Option in `isaacsim` package to launch isaac-sim.sh with custom args [Humble]

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
