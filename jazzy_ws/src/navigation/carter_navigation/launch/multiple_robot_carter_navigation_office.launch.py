# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch and rviz directories
    carter_nav2_bringup_dir = get_package_share_directory("carter_navigation")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")

    rviz_config_dir = os.path.join(carter_nav2_bringup_dir, "rviz2", "carter_navigation_namespaced.rviz")

    # Names and poses of the robots
    robots = [{"name": "carter1"}, {"name": "carter2"}, {"name": "carter3"}]

    # Common settings
    ENV_MAP_FILE = "carter_office_navigation.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml_file = LaunchConfiguration("map")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="True")

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(carter_nav2_bringup_dir, "maps", ENV_MAP_FILE),
        description="Full path to map file to load",
    )

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "carter1_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "office", "multi_robot_carter_navigation_params_1.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot1 launched nodes",
    )

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        "carter2_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "office", "multi_robot_carter_navigation_params_2.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot2 launched nodes",
    )

    declare_robot3_params_file_cmd = DeclareLaunchArgument(
        "carter3_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "office", "multi_robot_carter_navigation_params_3.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for robot3 launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the stacks"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_dir, description="Full path to the RVIZ config file to use."
    )

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot["name"] + "_params_file")

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot["name"]),
                        "use_namespace": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(carter_nav2_bringup_dir, "launch", "carter_navigation_individual.launch.py")
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": use_sim_time,
                        "params_file": params_file,
                        "default_bt_xml_filename": default_bt_xml_filename,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),
                
                Node(
                    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                    remappings=[('cloud_in', ['front_3d_lidar/lidar_points']),
                                ('scan', ['scan'])],
                    parameters=[{
                        'target_frame': 'front_3d_lidar',
                        'transform_tolerance': 0.01,
                        'min_height': -0.4,
                        'max_height': 1.5,
                        'angle_min': -1.5708,  # -M_PI/2
                        'angle_max': 1.5708,  # M_PI/2
                        'angle_increment': 0.0087,  # M_PI/360.0
                        'scan_time': 0.3333,
                        'range_min': 0.05,
                        'range_max': 100.0,
                        'use_inf': True,
                        'inf_epsilon': 1.0,
                        # 'concurrency_level': 1,
                    }],
                    name='pointcloud_to_laserscan',
                    namespace = robot["name"]
                ),

                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot["name"]]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " map yaml: ", map_yaml_file]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " params yaml: ", params_file]),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=[robot["name"], " behavior tree xml: ", default_bt_xml_filename],
                ),
                LogInfo(
                    condition=IfCondition(log_settings), msg=[robot["name"], " rviz config file: ", rviz_config_file]
                ),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " autostart: ", autostart]),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_robot3_params_file_cmd)

    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
