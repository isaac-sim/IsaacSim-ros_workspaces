#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
import argparse
import os
import subprocess
import signal
import sys
import atexit
import psutil

# Default values
defaults = {
    "isaac_sim_version": "5.1.0",
    "isaac_sim_path": "",
    "use_internal_libs": True,
    "dds_type": "fastdds",
    "gui": "",
    "standalone": "",
    "play_sim_on_start": False,
    "ros_distro_var": "humble",
    "ros_installation_path": "",
    "headless": "",
    "custom_args": "",
    "exclude_install_path": "",
}

# List to keep track of subprocesses
subprocesses = []

def signal_handler(sig, frame):
    print('Ctrl+C received, shutting down...')
    isaac_sim_shutdown()
    sys.exit(0)

def isaac_sim_shutdown():
    for proc_id in subprocesses:
        if sys.platform == "win32":
            os.kill(proc_id, signal.SIGTERM)
        else:
            os.killpg(os.getpgid(proc_id), signal.SIGKILL)
    print("All subprocesses terminated.")

# Register the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

def version_ge(v1, v2):
    return tuple(map(int, (v1.split(".")))) >= tuple(map(int, (v2.split("."))))

def version_gt(v1, v2):
    return tuple(map(int, (v1.split(".")))) > tuple(map(int, (v2.split("."))))

def update_env_vars(version_to_remove, specified_path_to_remove, env_var_name):
    env_var_value = os.environ.get(env_var_name, "")
    new_env_var_value = []
    for path in env_var_value.split(os.pathsep):
        if not version_to_remove in path and not path.startswith(specified_path_to_remove):
            new_env_var_value.append(path)
    os.environ[env_var_name] = os.pathsep.join(new_env_var_value)

def exclude_paths_from_env(exclude_paths_str, env_var_name):
    """Simple function to exclude paths from environment variable"""
    if not exclude_paths_str:
        return
    
    env_var_value = os.environ.get(env_var_name, "")
    if not env_var_value:
        return
    
    exclude_paths = [path.strip() for path in exclude_paths_str.split(',') if path.strip()]
    paths = env_var_value.split(os.pathsep)
    filtered_paths = []
    
    for path in paths:
        exclude_this_path = False
        for exclude_path in exclude_paths:
            if exclude_path in path:
                exclude_this_path = True
                break
        if not exclude_this_path:
            filtered_paths.append(path)
    
    os.environ[env_var_name] = os.pathsep.join(filtered_paths)

class IsaacSimLauncherNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_launcher_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('version', defaults['isaac_sim_version']),
                ('install_path', defaults['isaac_sim_path']),
                ('use_internal_libs', defaults['use_internal_libs']),
                ('dds_type', defaults['dds_type']),
                ('gui', defaults['gui']),
                ('standalone', defaults['standalone']),
                ('play_sim_on_start', defaults['play_sim_on_start']),
                ('ros_distro', defaults['ros_distro_var']),
                ('ros_installation_path', defaults['ros_installation_path']),
                ('headless', defaults['headless']),
                ('custom_args', defaults['custom_args']),
                ('exclude_install_path', defaults['exclude_install_path'])
            ]
        )
        self.execute_launch()

    def execute_launch(self):
        args = argparse.Namespace()
        args.version = self.get_parameter('version').get_parameter_value().string_value
        args.install_path = self.get_parameter('install_path').get_parameter_value().string_value
        args.use_internal_libs = self.get_parameter('use_internal_libs').get_parameter_value().bool_value
        args.dds_type = self.get_parameter('dds_type').get_parameter_value().string_value
        args.gui = self.get_parameter('gui').get_parameter_value().string_value
        args.standalone = self.get_parameter('standalone').get_parameter_value().string_value
        args.play_sim_on_start = self.get_parameter('play_sim_on_start').get_parameter_value().bool_value
        args.ros_distro = self.get_parameter('ros_distro').get_parameter_value().string_value
        args.ros_installation_path = self.get_parameter('ros_installation_path').get_parameter_value().string_value
        args.headless = self.get_parameter('headless').get_parameter_value().string_value
        args.custom_args = self.get_parameter('custom_args').get_parameter_value().string_value
        args.exclude_install_path = self.get_parameter('exclude_install_path').get_parameter_value().string_value

        filepath_root = ""

        if args.install_path != "":
            filepath_root = args.install_path
        else:
            # If custom Isaac Sim Installation folder not given, use the default path using version number provided.
            home_var = "USERPROFILE" if sys.platform == "win32" else "HOME"
            home_path = os.getenv(home_var)
            if version_ge(args.version, "4.2.0") and not version_gt(args.version, "2021.2.0"):
                if sys.platform == "win32":
                    filepath_root = os.path.join("C:", "isaacsim")
                else:
                    filepath_root = os.path.join(home_path, "isaacsim")
            elif args.version == "4.2.0":
                if sys.platform == "win32":
                    filepath_root = os.path.join(home_path, "AppData", "Local", "ov", "pkg", f"isaac-sim-{args.version}")
                else:
                    filepath_root = os.path.join(home_path, ".local", "share", "ov", "pkg", f"isaac-sim-{args.version}")
            elif version_ge(args.version, "2021.2.1") and not version_ge(args.version, "2023.1.2"):
                if sys.platform == "win32":
                    filepath_root = os.path.join(home_path, "AppData", "Local", "ov", "pkg", f"isaac_sim-{args.version}")
                else:
                    filepath_root = os.path.join(home_path, ".local", "share", "ov", "pkg", f"isaac_sim-{args.version}")
            else:
                print(f"Unsupported Isaac Sim version: {args.version}")
                sys.exit(0)

        os.environ["ROS_DISTRO"] = args.ros_distro

        if args.use_internal_libs:
            if sys.platform == "win32":
                print("use_internal_libs parameter is not supported in Windows")
                sys.exit(0)
            else:
                os.environ["LD_LIBRARY_PATH"] = f"{os.getenv('LD_LIBRARY_PATH')}:{filepath_root}/exts/isaacsim.ros2.bridge/{args.ros_distro}/lib"
                specific_path_to_remove = f"/opt/ros/{args.ros_distro}"
                version_to_remove = "jazzy" if args.ros_distro == "humble" else "humble"
                update_env_vars(version_to_remove, specific_path_to_remove, "LD_LIBRARY_PATH")
                update_env_vars(version_to_remove, specific_path_to_remove, "PYTHONPATH")
                update_env_vars(version_to_remove, specific_path_to_remove, "PATH")
        
        # Apply path exclusions AFTER all other modifications
        if args.exclude_install_path:
            exclude_paths_from_env(args.exclude_install_path, "LD_LIBRARY_PATH")
            exclude_paths_from_env(args.exclude_install_path, "PYTHONPATH")
            exclude_paths_from_env(args.exclude_install_path, "PATH")
        
        if args.ros_installation_path:
            # If a custom ros installation path is provided (can be comma-separated list)
            if sys.platform == "win32":
                print("Custom ROS installation path not fully supported on Windows")
            else:
                # Split by comma to handle multiple paths
                ros_paths = [path.strip() for path in args.ros_installation_path.split(',') if path.strip()]
                
                for ros_path in ros_paths:
                    # Check if it's a setup.bash file or a directory
                    if ros_path.endswith('setup.bash') or 'setup.bash' in ros_path:
                        # It's a ROS installation setup file
                        source_cmd = f"source {ros_path} && env"
                        result = subprocess.run(['bash', '-c', source_cmd], 
                                                capture_output=True, text=True, check=True)
                        
                        # Parse and apply environment variables
                        for line in result.stdout.splitlines():
                            if '=' in line:
                                key, value = line.split('=', 1)
                                if key in ['LD_LIBRARY_PATH', 'PYTHONPATH', 'PATH', 'ROS_DISTRO']:
                                    os.environ[key] = value
                    else:
                        # It's a workspace install directory - add to environment variables
                        install_path = ros_path.rstrip('/')
                        
                        # Add to LD_LIBRARY_PATH
                        current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
                        if current_ld_path:
                            os.environ['LD_LIBRARY_PATH'] = f"{install_path}/lib:{current_ld_path}"
                        else:
                            os.environ['LD_LIBRARY_PATH'] = f"{install_path}/lib"
                        
                        # Add to PYTHONPATH
                        current_python_path = os.environ.get('PYTHONPATH', '')
                        if current_python_path:
                            os.environ['PYTHONPATH'] = f"{install_path}/lib/python3/dist-packages:{current_python_path}"
                        else:
                            os.environ['PYTHONPATH'] = f"{install_path}/lib/python3/dist-packages"

        os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp" if args.dds_type == "cyclonedds" else "rmw_fastrtps_cpp"
        play_sim_on_start_arg = "--start-on-play" if args.play_sim_on_start else ""

        if args.standalone != "":
            executable_path = os.path.join(filepath_root, "python.sh" if sys.platform != "win32" else "python.bat")
            proc = subprocess.Popen(f"{executable_path} {args.standalone}", shell=True, start_new_session=True)
            subprocesses.append(proc.pid)
        else:
            # Default command
            executable_command = f'{os.path.join(filepath_root, "isaac-sim.sh" if sys.platform != "win32" else "isaac-sim.bat")} --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'

            if args.headless == "webrtc":
                executable_command = f'{os.path.join(filepath_root, "isaac-sim.streaming.sh" if sys.platform != "win32" else "isaac-sim.streaming.bat")} --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'

            if args.custom_args!= "":
                executable_command += f" {args.custom_args}"

            if args.gui != "":
                script_dir = os.path.dirname(__file__)
                file_arg = os.path.join(script_dir, "open_isaacsim_stage.py") + f" --path {args.gui} {play_sim_on_start_arg}"
                executable_command += f" --exec '{file_arg}'"

            proc = subprocess.Popen(executable_command, shell=True, start_new_session=True)
            subprocesses.append(proc.pid)

def main(args=None):
    rclpy.init(args=args)
    isaac_sim_launcher_node = IsaacSimLauncherNode()
    rclpy.spin(isaac_sim_launcher_node)
    # Ensure all subprocesses are terminated before exiting
    isaac_sim_shutdown()
    isaac_sim_launcher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
