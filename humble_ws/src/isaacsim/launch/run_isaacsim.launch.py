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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


# Declare all launch arguments corresponding to the bash script options
launch_args = [
    DeclareLaunchArgument('version', default_value='5.1.0', description='Specify the version of Isaac Sim to use. Isaac Sim will be run from default install root folder for the specified version. Leave empty to use latest version of Isaac Sim.'),
    
    DeclareLaunchArgument('install_path', default_value='', description='If Isaac Sim is insalled in a non-default location, provide a specific path to Isaac Sim installation root folder. (If defined, "version" parameter will be ignored)'),
    
    DeclareLaunchArgument('use_internal_libs', default_value='true', description='Set to true if you wish to use internal ROS libraries shipped with Isaac Sim.'),
    
    DeclareLaunchArgument('dds_type', default_value='fastdds', description='Set to "fastdds" or "cyclonedds" (Cyclone only supported for ROS 2 Humble and ROS 2 Jazzy) to run Isaac Sim with a specific dds type.'),
    
    DeclareLaunchArgument('gui', default_value='', description='Provide the path to a usd file to open it when starting Isaac Sim in standard gui mode. If left empty, Isaac Sim will open an empty stage in standard gui mode.'),
    
    DeclareLaunchArgument('standalone', default_value='', description='Provide the path to the python file to open it and start Isaac Sim in standalone workflow. If left empty, Isaac Sim will open an empty stage in standard Gui mode.'),
    
    DeclareLaunchArgument('play_sim_on_start', default_value='false', description='If enabled and Isaac Sim will start playing the scene after it is loaded. (Only applicable when in standard gui mode and loading a scene)'),
    
    DeclareLaunchArgument('ros_distro', default_value='humble', description='Provide ROS version to use. Only Humble and Jazzy is supported.'),
    
    DeclareLaunchArgument('ros_installation_path', default_value='', description='Comma-separated list of ROS installation paths. If ROS is installed in a non-default location (as in not under /opt/ros/), provide the path to your main setup.bash file for your ROS install. (/path/to/custom/ros/install/setup.bash). Similarly add the path to your local_setup.bash file for your workspace installation. (/path/to/custom_ros_workspace/install/local_setup.bash)'),

    DeclareLaunchArgument('headless', default_value='', description='Set to "native" or "webrtc" to run Isaac Sim with different headless modes, if left blank, Isaac Sim will run in the regular GUI workflow. This parameter can be overridden by "standalone" parameter.'),

    DeclareLaunchArgument('custom_args', default_value='', description='Add any custom Isaac Sim args that you want to forward to isaac-sim.sh during run time.'),

    DeclareLaunchArgument('exclude_install_path', default_value='', description='Comma-separated list of installation paths to exclude from LD_LIBRARY_PATH, PYTHONPATH, and PATH environment variables.'),

]

def launch_setup(context):
        
    # Run isaac sim as a ROS2 node with default parameters. Parameters can be overridden here or via launch arguments from other launch files. 
    isaacsim_node = Node(
        package='isaacsim', executable='run_isaacsim.py',
        name='isaacsim', output="screen", 
        parameters=[{
            'version': LaunchConfiguration('version'),
            'install_path': LaunchConfiguration('install_path'),
            'use_internal_libs': LaunchConfiguration('use_internal_libs'),
            'dds_type': LaunchConfiguration('dds_type'),
            'gui': LaunchConfiguration('gui'),
            'standalone': LaunchConfiguration('standalone'),
            'play_sim_on_start': LaunchConfiguration('play_sim_on_start'),
            'ros_distro': LaunchConfiguration('ros_distro'),
            'ros_installation_path': LaunchConfiguration('ros_installation_path'),
            'headless': LaunchConfiguration('headless'),
            'custom_args': LaunchConfiguration('custom_args'),
            'exclude_install_path': LaunchConfiguration('exclude_install_path')
        }]
    )
    return [isaacsim_node]


# Create and return the launch description with all declared arguments and the execute launch_setup
def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
