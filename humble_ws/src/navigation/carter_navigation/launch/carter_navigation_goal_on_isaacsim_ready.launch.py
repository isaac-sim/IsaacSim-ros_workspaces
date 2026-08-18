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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ISAAC_SIM_READY_MESSAGES = (
    "Stage loaded and simulation is playing.",
    "Isaac Sim Full App is loaded.",
)


def generate_launch_description():
    navigation_goal_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("isaac_ros_navigation_goal"),
                "launch",
                "isaac_ros_navigation_goal.launch.xml",
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("goal_namespace"),
            "map_yaml_path": LaunchConfiguration("map_yaml_path"),
            "goal_text_file_path": LaunchConfiguration("goal_text_file_path"),
            "iteration_count": LaunchConfiguration("iteration_count"),
            "goal_generator_type": LaunchConfiguration("goal_generator_type"),
            "action_server_name": LaunchConfiguration("action_server_name"),
            "obstacle_search_distance_in_meters": LaunchConfiguration("obstacle_search_distance_in_meters"),
            "frame_id": LaunchConfiguration("frame_id"),
            "initial_pose": LaunchConfiguration("initial_pose"),
            "action_server_timeout_sec": LaunchConfiguration("action_server_timeout_sec"),
            "initial_pose_subscriber_timeout_sec": LaunchConfiguration("initial_pose_subscriber_timeout_sec"),
            "initial_pose_settle_time_sec": LaunchConfiguration("initial_pose_settle_time_sec"),
            "lifecycle_node_name": LaunchConfiguration("lifecycle_node_name"),
            "lifecycle_state_timeout_sec": LaunchConfiguration("lifecycle_state_timeout_sec"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    automatic_goal_launched = False

    def launch_navigation_goal(reason):
        nonlocal automatic_goal_launched
        if automatic_goal_launched:
            return None

        automatic_goal_launched = True
        return [
            LogInfo(msg=f"Launching automatic navigation goals: {reason}."),
            navigation_goal_launch,
        ]

    def launch_navigation_goal_when_ready(event):
        if isinstance(event.text, bytes):
            output = event.text.decode(errors="replace")
        else:
            output = str(event.text)

        for ready_message in ISAAC_SIM_READY_MESSAGES:
            if ready_message in output:
                return launch_navigation_goal(f"Isaac Sim reported '{ready_message}'")

        return None

    def launch_navigation_goal_after_timeout(context):
        return launch_navigation_goal("startup timeout fallback")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "goal_namespace",
                default_value="",
                description="ROS namespace for the goal node and its relative interfaces.",
            ),
            DeclareLaunchArgument(
                "map_yaml_path",
                default_value=os.path.join(
                    get_package_share_directory("isaac_ros_navigation_goal"),
                    "assets",
                    "carter_warehouse_navigation.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "goal_text_file_path",
                default_value=os.path.join(
                    get_package_share_directory("isaac_ros_navigation_goal"),
                    "assets",
                    "goals.txt",
                ),
            ),
            DeclareLaunchArgument("iteration_count", default_value="3"),
            DeclareLaunchArgument("goal_generator_type", default_value="RandomGoalGenerator"),
            DeclareLaunchArgument("action_server_name", default_value="navigate_to_pose"),
            DeclareLaunchArgument("obstacle_search_distance_in_meters", default_value="0.2"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument(
                "initial_pose",
                default_value="[-6.4,-1.04,0.0,0.0,0.0,0.99,0.02]",
            ),
            DeclareLaunchArgument("action_server_timeout_sec", default_value="180.0"),
            DeclareLaunchArgument(
                "initial_pose_subscriber_timeout_sec",
                default_value="180.0",
            ),
            DeclareLaunchArgument("initial_pose_settle_time_sec", default_value="10.0"),
            DeclareLaunchArgument("lifecycle_node_name", default_value="bt_navigator"),
            DeclareLaunchArgument("lifecycle_state_timeout_sec", default_value="180.0"),
            DeclareLaunchArgument(
                "goal_start_timeout_sec",
                default_value="45.0",
                description=("Fallback delay before starting goals if no Isaac Sim ready log appears."),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Isaac Sim) clock for stamped poses.",
            ),
            RegisterEventHandler(
                OnProcessIO(
                    on_stdout=launch_navigation_goal_when_ready,
                    on_stderr=launch_navigation_goal_when_ready,
                )
            ),
            TimerAction(
                period=LaunchConfiguration("goal_start_timeout_sec"),
                actions=[OpaqueFunction(function=launch_navigation_goal_after_timeout)],
            ),
        ]
    )
