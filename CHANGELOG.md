# Changelog

## [6.10.0] - 2026-09-02

### Security

- Updated the Windows Pixi environments to OpenSSL 3.6.4. [Humble, Jazzy]

## [6.9.0] - 2026-09-01
### Added
- Added CI matrix coverage for every supported `build_ros.sh` ROS 2 and Ubuntu combination. [Humble, Jazzy]

### Fixed
- Installed CA certificates, modernized the ROS apt keyring setup, and included missing workspace dependencies in the Jazzy Docker images. [Jazzy]
- Serialized Docker workspace source imports to avoid intermittent GitHub throttling in CI. [Humble, Jazzy]

## [6.8.4] - 2026-08-27
### Fixed
- `carter_navigation`: Corrected the Humble SLAM launch argument Boolean so multi-robot navigation launches no longer fail during Nav2 Python expression evaluation. [Humble]

## [6.8.3] - 2026-08-27
### Fixed
- `isaacsim_clearpath_nav2`: Added localization-only scan filtering so AMCL ignores severely incomplete lidar scans that can destabilize localization during in-place rotations. [Jazzy]

## [6.8.2] - 2026-08-25
### Fixed
- `h1_fullbody_controller`: Removed the duplicate `use_sim_time` declaration that prevented the controller from starting. [Humble, Jazzy]

## [6.8.1] - 2026-08-24
### Changed
- Bump versions to 6.1.0 [Humble, Jazzy]

## [6.8.0] - 2026-08-24
### Fixed
- `isaacsim_bringup`: Isolated Isaac Sim child processes from Pixi and ROS activation paths while preserving unrelated user environment settings. [Humble, Jazzy]

## [6.7.0] - 2026-08-22
### Added
- Added `pal_statistics`, `ros2_control`, and `ros2_control_cmake` as workspace submodules. [Jazzy]

## [6.6.1] - 2026-08-17
### Fixed
- Navigation goals wait for Isaac Sim/Nav2. [Humble, Jazzy]

## [6.6.0] - 2026-08-05
### Changed
- Removed the `standalone` Pixi task. Run standalone scripts with Isaac Sim's bundled Python launcher from a clean terminal to avoid exposing the Pixi dependency environment to Isaac Sim. [Humble, Jazzy]
- `isaac_ros2_control_demo`: Installed `add_floor` as a ROS console script and removed redundant direct script installs. [Humble, Jazzy]
- `isaac_ros2_control_demo`: Replaced stdout capture of `/robot_description` with a one-shot node and atomic parameter-file handoff so middleware logs cannot corrupt the URDF. [Humble, Jazzy]
- `isaac_ros2_control_demo`: Wait indefinitely for `/robot_description`, reporting the wait every 60 seconds until the message arrives or the user presses `Ctrl+C`. [Humble, Jazzy]

## [6.5.0] - 2026-08-04
### Changed
- `isaacsim_bringup`: Updated the default Isaac Sim launch version and package documentation links to 6.1.0. [Humble, Jazzy]

### Fixed
- `cmdvel_to_ackermann`: Restored reliable `cmd_vel` to `ackermann_cmd` conversion and publishing by using relative/configurable topics, stamping every published `AckermannDriveStamped` message, setting the default Ackermann frame ID, and using wheelbase for steering conversion. [Humble, Jazzy]

## [6.4.1] - 2026-08-04
- `isaac_ros2_control_demo`: Installed `read_robot_description` and `add_floor` as ROS console scripts so launch can discover their `.exe` wrappers on Windows, and enabled the floor collision node for Humble. [Humble, Jazzy]

## [6.4.0] - 2026-07-29
### Added
- Added Greenwave Monitor as a workspace submodule with Pixi build dependencies and setup documentation. [Humble, Jazzy]

## [6.3.0] - 2026-07-20
### Added
- `isaac_compressed_image_decoder`: auto-discovers `sensor_msgs/msg/CompressedImage` topics, decodes H.264 and HEVC/H.265 payloads based on the message `format`, and republishes `/topic/compressed` as `/topic` raw `sensor_msgs/Image` topics. Existing single-topic `input_topic`/`output_topic` parameters remain supported. [Humble, Jazzy]

### Fixed
- `isaac_ros2_control_demo`: Lowered the MoveIt floor collision surface by 5 cm to prevent the UR10 default pose from starting in collision. [Jazzy]

## [6.2.0] - 2026-07-13
### Added
- `isaacsim_bringup`: Added `python_script` launch argument to run a user-provided Python script inside Isaac Sim
  after startup in GUI/headless mode. [Humble, Jazzy]
- `isaacsim_bringup`: Added an installed `add_cube_and_lights.py` sample startup script. [Humble, Jazzy]
- Added the `isaac_ros2_control_demo` package for controlling a UR10 in Isaac Sim with MoveIt 2 and an in-process ROS 2 Controller Manager. [Humble, Jazzy]
- Added a Pixi environment and lockfile for the Humble workspace. [Humble]
- Added Linux AArch64 as a supported Pixi platform. [Humble, Jazzy]

### Changed
- Migrated all repository-owned ROS 2 launch files from Python to XML and updated their package metadata and documentation references. [Humble, Jazzy]
- Added the RViz Visual Tools dependency to the Jazzy Pixi environment. [Jazzy]
- Updated `.gitignore` to ignore Python bytecode, `__pycache__` directories, and editor backup files.

### Fixed
- `isaac_ros_navigation_goal`: Wait for Nav2 and localization readiness before sending goals, forward map configuration from integrated navigation launches, and use namespace-relative action and `initialpose` names for multi-robot launches. [Humble, Jazzy]
- `isaac_ros_navigation_goal` and `cmdvel_to_ackermann`: Use the simulation clock for stamped navigation poses and Ackermann commands, with launch arguments for explicit clock selection. [Humble, Jazzy]
- `isaac_ros_navigation_goal`: Preserve the initial pose as a typed floating-point array when passed through the XML launch frontend. [Humble, Jazzy]
- `carter_navigation`: Forward `use_sim_time` to the point-cloud-to-laser-scan node. [Humble, Jazzy]
- Navigation launches: Forward `use_sim_time` to Jazzy RViz instances and Nova Carter's robot-state publisher. [Humble, Jazzy where applicable]

## [6.1.1] - 2026-07-02
### Added
- Added repo-level Python/C++ formatter tooling using isort, Black, and clang-format.

### Changed
- Updated MoveIt-related submodules with formatting-only changes: `moveit_resources` [Humble, Jazzy] and `topic_based_ros2_control` [Jazzy].

## [6.1.0] - 2026-06-23
### Changed
- Bumped versions for Isaac Sim pip dependency to 6.0.1.0.

## [6.0.3] - 2026-06-10
### Changed
- Bumped versions for Isaac Sim pip dependency to 6.0.0.1.
- Bumped versions for torch pip dependency to 2.11.0.

## [6.0.3] - 2026-06-09
### Changed
- Pixi toml now adds Isaac Sim pip dependency.

## [6.0.2] - 2026-05-19
### Fixed
- `isaac_moveit`: Added local `panda_isaac.urdf.xacro` that omits `PandaHandFakeSystem` to stop malformed JointState warnings on `/isaac_joint_commands` and the mimic-loop fault that destabilized `panda_arm_controller`. Extended `gripper_to_isaac.py` to forward finger positions from `/isaac_joint_states` to `/joint_states` for MoveIt's planning scene. [Jazzy]

## [6.0.1] - 2026-05-18
### Fixed
- Added pip dependency in package.xml for h1_fullbody_controller package [Jazzy]

## [6.0.0] - 2026-05-11
### Changed
- Renamed `isaacsim` package to `isaacsim_bringup` in both Humble and Jazzy workspaces [Humble, Jazzy]

## [5.3.0] - 2026-05-06
### Fixed
- `run_isaacsim.py`: Fixed `--exec` quoting on Windows so the `gui:=` USD path opens correctly under pixi. cmd.exe does not strip single quotes, so the launcher now uses double quotes on Windows. [Humble, Jazzy]
- `run_isaacsim.py`: Replaced POSIX-only `start_new_session=True` with `creationflags=CREATE_NEW_PROCESS_GROUP` on Windows for the Isaac Sim subprocess. [Humble, Jazzy]
- `run_isaacsim.py`: `use_internal_libs` and `ros_installation_path` now exit with a non-zero status and a clear error on Windows instead of silently exiting 0 or continuing. [Humble, Jazzy]

### Changed
- `run_isaacsim.py`: `dds_type` is now opt-in and defaults to empty. When empty, the surrounding `RMW_IMPLEMENTATION` (e.g. `rmw_zenoh_cpp` set by pixi activation) is preserved. Explicit values map to `fastdds`/`cyclonedds`/`zenoh`. [Humble, Jazzy]
- `run_isaacsim.py`: When `install_path:=` is not provided, the launcher falls back to the `isaac_sim_package_path` environment variable (set by pixi activation) before the version-based default. [Humble, Jazzy]

## [5.2.0] - 2026-05-01
### Changed
- `use_internal_libs` default changed from `True` to `False` in the `isaacsim` package [Jazzy]. Python 3.12 on Isaac Sim allows ROS 2 system install to be sourced directly from the system, making internal lib loading unnecessary. Humble retains the default of `True`. Jazzy users who explicitly relied on internal libs should set `use_internal_libs:=True` when launching and verify their integration against Isaac Sim 6.0.
- Converted `isaacsim`, `cmdvel_to_ackermann`, `h1_fullbody_controller`, and `isaac_moveit` packages from `ament_cmake` to `ament_python` build type [Humble, Jazzy]
- Fixed `open_isaacsim_stage.py` path resolution to use `get_package_share_directory` instead of `__file__` [Humble, Jazzy]

## [5.1.0] - 2026-03-05
### Changed
- Added custom `panda_isaac.urdf.xacro` and `gripper_to_isaac.py` bridge for improved MoveIt Isaac Sim performance. [Humble]
- Added rmw_zenoh support for Jazzy 22.04 Docker build: conditional Rust toolchain install, additional rosinstall_generator deps (tinyxml2_vendor, rmw_dds_common, fastcdr, rosidl_typesupport_fastrtps_c/cpp, rclcpp), nlohmann-json3-dev, and suppressed unused CMake variable warnings. [Jazzy]
- Added `--no-cache` (`-n`) flag to `build_ros.sh` for Docker cache-free rebuilds. [Humble, Jazzy]

## [5.0.0] - 2025-12-10
### Added
- Ubuntu 24.04/ROS 2 Jazzy Python 3.12 build support and new dockerfile. [Humble, Jazzy]
- Ubuntu 22.04/ROS 2 Humble and Jazzy Python 3.12 build support and new dockerfiles. [Humble, Jazzy]
- Added topic_based_ros2_control ros2 package as submodule to workspace [Jazzy] 

### Changed
- Bumped versions to 6.0.
- Cleaned up occupancy map parameters in Navigation packages. [Humble, Jazzy]
- Updated helper script `build_ros.sh` to support new dockerfiles.
- Updated internal libraries path to isaacsim.ros2.core in isaacsim package [Humble, Jazzy]

### Removed
- Legacy references to older Ubuntu/Python/ROS mentions from launch files, parameters and build scripts.
- Redundant or broken dependencies from docker build stages.


## [4.7.0] - 2025-12-02
### Added
- isaac_tutorials.ros2_object_id_subscriber example [Humble, Jazzy]

## [4.6.0] - 2025-11-25
### Changed
- Updated `h1_fullbody_controller` launch to add `namespace` argument to enable multi-humanoid [Humble, Jazzy]
- Switched all topics in `h1_fullbody_controller` to relative names to support namespaced multi-humanoid setups [Humble, Jazzy]

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
