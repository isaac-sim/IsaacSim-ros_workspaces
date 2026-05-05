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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    pkg_dir = get_package_share_directory("isaacsim_clearpath_nav2")
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    setup_path = LaunchConfiguration("setup_path")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    robot_urdf = PathJoinSubstitution([setup_path, "robot.urdf.xacro"])
    config_control = PathJoinSubstitution([setup_path, "platform", "config", "control.yaml"])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", robot_urdf,
            " is_sim:=true",
            " gazebo_controllers:=", config_control,
            " namespace:=", namespace,
            " use_fake_hardware:=true",
            " use_platform_controllers:=false",
            " use_manipulation_controllers:=false",
        ]),
        value_type=str,
    )

    rewritten_parameters = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            # Explicitly rewrite scan topic since local_costmap/global_costmap nodes prepend their
            # own sub-namespace. 
            'topic': ['/', namespace, '/sim_lidar/scan'],
        },
        convert_types=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "setup_path",
                default_value=os.path.join(pkg_dir, "params", "dd100"),
                description="Path to clearpath-generated description directory (must contain robot.urdf.xacro from generate_description)",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace",
            ),
            DeclareLaunchArgument(
                "map",
                description="Full path to the occupancy-map yaml file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([setup_path, "nav2.yaml"]),
                description="Full path to the Nav2 parameters file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Omniverse Isaac Sim) clock if true",
            ),
            # Robot state publisher (xacro -> URDF -> /robot_description + /tf_static)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                parameters=[{
                    "robot_description": robot_description_content,
                    "use_sim_time": use_sim_time,
                }],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
            # RViz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": "true",
                    "rviz_config": os.path.join(
                        pkg_dir, "params", "rviz2", "isaacsim_clearpath_nav2_namespaced.rviz"
                    ),
                }.items(),
            ),
            # Nav2 full bringup (localization, navigation, map server, etc.)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": rewritten_parameters,
                }.items(),
            ),
        ]
    )
