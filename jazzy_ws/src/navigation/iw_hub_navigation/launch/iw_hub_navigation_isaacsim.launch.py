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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.substitutions import FindExecutable

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"), "maps", "iw_hub_warehouse_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("iw_hub_navigation"), "params", "iw_hub_navigation_params.yaml"
        ),
    )


    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("iw_hub_navigation"), "rviz2", "iw_hub_navigation.rviz")

    ld_automatic_goal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("isaac_ros_navigation_goal"), "launch", "isaac_ros_navigation_goal.launch.py"
                ),
            ]
        ),
    )

    

    def execute_second_node_if_condition_met(event, second_node_action):
        output = event.text.decode().strip()
        # Look for fully loaded message from Isaac Sim. Only applicable in Gui mode.
        if "Stage loaded and simulation is playing." in output:
            # Log a message indicating the condition has been met
            print("Condition met, launching the second node.")
            
            # If Nav2 takes additional time to initialize, uncomment the lines below to add a delay of 10 seconds (or any desired duration) before launching the second_node_action
            # import time
            # time.sleep(10)
            return second_node_action


    return LaunchDescription(
        [
            # Declaring the Isaac Sim scene path. 'gui' launch argument is already used withing run_isaac_sim.launch.py
            DeclareLaunchArgument("gui", default_value='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Samples/ROS2/Scenario/iw_hub_warehouse_navigation.usd', description="Path to isaac sim scene"),

            # Include Isaac Sim launch file from isaacsim package with given launch parameters.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory("isaacsim"), "launch", "run_isaacsim.launch.py"
                        ),
                    ]
                ),
                launch_arguments={
                    'version': '5.1.0',
                    'play_sim_on_start': 'True',
                }.items(),
            ),
            
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="True", description="Use simulation (Omniverse Isaac Sim) clock if True"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir}.items(),
            ),
            # Launch automatic goal generator node when Isaac Sim has finished loading.
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=lambda event: execute_second_node_if_condition_met(event, ld_automatic_goal)
                )
            ),
        ]
    )
