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

import argparse
import atexit
import os
import shlex
import signal
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# Default values
defaults = {
    "isaac_sim_version": "6.1.0",
    "isaac_sim_path": "",
    "use_internal_libs": True,
    "dds_type": "",
    "gui": "",
    "standalone": "",
    "python_script": "",
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
    print("Ctrl+C received, shutting down...")
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


def update_env_vars(version_to_remove, specified_path_to_remove, env_var_name, environment=None):
    environment = os.environ if environment is None else environment
    env_var_value = environment.get(env_var_name, "")
    new_env_var_value = []
    for path in env_var_value.split(os.pathsep):
        if not version_to_remove in path and not path.startswith(specified_path_to_remove):
            new_env_var_value.append(path)
    environment[env_var_name] = os.pathsep.join(new_env_var_value)


def exclude_paths_from_env(exclude_paths_str, env_var_name, environment=None):
    """Simple function to exclude paths from environment variable"""
    environment = os.environ if environment is None else environment
    if not exclude_paths_str:
        return

    env_var_value = environment.get(env_var_name, "")
    if not env_var_value:
        return

    exclude_paths = [path.strip() for path in exclude_paths_str.split(",") if path.strip()]
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

    environment[env_var_name] = os.pathsep.join(filtered_paths)


def _is_path_under_roots(path, roots):
    if not path:
        return False

    candidate = os.path.normcase(os.path.realpath(os.path.expanduser(os.path.expandvars(path.strip('"')))))
    for root in roots:
        try:
            if os.path.commonpath([candidate, root]) == root:
                return True
        except ValueError:
            continue
    return False


def _create_isaac_sim_environment(source_environment=None):
    """Remove Pixi activation from the environment inherited by Isaac Sim."""
    environment = dict(os.environ if source_environment is None else source_environment)
    roots = []
    for variable in ("CONDA_PREFIX", "PIXI_PROJECT_ROOT"):
        root = environment.get(variable)
        if root:
            roots.append(os.path.normcase(os.path.realpath(os.path.expanduser(os.path.expandvars(root)))))

    if not roots:
        return environment

    for variable in (
        "PATH",
        "PYTHONPATH",
        "LD_LIBRARY_PATH",
        "LD_PRELOAD",
        "PKG_CONFIG_PATH",
        "QT_PLUGIN_PATH",
        "QML2_IMPORT_PATH",
    ):
        value = environment.get(variable)
        if not value:
            continue
        filtered = [entry for entry in value.split(os.pathsep) if not _is_path_under_roots(entry, roots)]
        if filtered:
            environment[variable] = os.pathsep.join(filtered)
        else:
            environment.pop(variable, None)

    for variable in (
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "PYTHONHOME",
        "RMW_IMPLEMENTATION",
        "ROS_DISTRO",
        "ROS_PYTHON_VERSION",
        "ROS_VERSION",
        "VIRTUAL_ENV",
    ):
        environment.pop(variable, None)

    for variable in list(environment):
        if variable.startswith(("AMENT_", "COLCON_", "CONDA_", "PIXI_")):
            environment.pop(variable)

    return environment


def _build_exec_command(command_args):
    if sys.platform == "win32":
        return subprocess.list2cmdline(command_args)
    return shlex.join(command_args)


def _quote_command_argument(command_arg):
    if sys.platform == "win32":
        return subprocess.list2cmdline([command_arg])
    return shlex.quote(command_arg)


def _resolve_python_script_path(python_script):
    if not python_script:
        return ""

    script_path = os.path.abspath(os.path.expanduser(python_script))
    if not os.path.isfile(script_path):
        print(f"ERROR: python_script path does not exist or is not a file: {script_path}", file=sys.stderr)
        sys.exit(1)

    return script_path


class IsaacSimLauncherNode(Node):
    def __init__(self):
        super().__init__("isaac_sim_launcher_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("version", defaults["isaac_sim_version"]),
                ("install_path", defaults["isaac_sim_path"]),
                ("use_internal_libs", defaults["use_internal_libs"]),
                ("dds_type", defaults["dds_type"]),
                ("gui", defaults["gui"]),
                ("standalone", defaults["standalone"]),
                ("python_script", defaults["python_script"]),
                ("play_sim_on_start", defaults["play_sim_on_start"]),
                ("ros_distro", defaults["ros_distro_var"]),
                ("ros_installation_path", defaults["ros_installation_path"]),
                ("headless", defaults["headless"]),
                ("custom_args", defaults["custom_args"]),
                ("exclude_install_path", defaults["exclude_install_path"]),
            ],
        )
        self.execute_launch()

    def execute_launch(self):
        args = argparse.Namespace()
        args.version = self.get_parameter("version").get_parameter_value().string_value
        args.install_path = self.get_parameter("install_path").get_parameter_value().string_value
        args.use_internal_libs = self.get_parameter("use_internal_libs").get_parameter_value().bool_value
        args.dds_type = self.get_parameter("dds_type").get_parameter_value().string_value
        args.gui = self.get_parameter("gui").get_parameter_value().string_value
        args.standalone = self.get_parameter("standalone").get_parameter_value().string_value
        args.python_script = self.get_parameter("python_script").get_parameter_value().string_value
        args.play_sim_on_start = self.get_parameter("play_sim_on_start").get_parameter_value().bool_value
        args.ros_distro = self.get_parameter("ros_distro").get_parameter_value().string_value
        args.ros_installation_path = self.get_parameter("ros_installation_path").get_parameter_value().string_value
        args.headless = self.get_parameter("headless").get_parameter_value().string_value
        args.custom_args = self.get_parameter("custom_args").get_parameter_value().string_value
        args.exclude_install_path = self.get_parameter("exclude_install_path").get_parameter_value().string_value

        filepath_root = ""

        if args.install_path != "":
            filepath_root = os.path.expanduser(args.install_path)
        elif os.environ.get("isaac_sim_package_path"):
            filepath_root = os.path.expanduser(os.environ["isaac_sim_package_path"])
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
                    filepath_root = os.path.join(
                        home_path, "AppData", "Local", "ov", "pkg", f"isaac-sim-{args.version}"
                    )
                else:
                    filepath_root = os.path.join(home_path, ".local", "share", "ov", "pkg", f"isaac-sim-{args.version}")
            elif version_ge(args.version, "2021.2.1") and not version_ge(args.version, "2023.1.2"):
                if sys.platform == "win32":
                    filepath_root = os.path.join(
                        home_path, "AppData", "Local", "ov", "pkg", f"isaac_sim-{args.version}"
                    )
                else:
                    filepath_root = os.path.join(home_path, ".local", "share", "ov", "pkg", f"isaac_sim-{args.version}")
            else:
                print(f"Unsupported Isaac Sim version: {args.version}")
                sys.exit(0)

        is_pixi_environment = bool(os.environ.get("CONDA_PREFIX") or os.environ.get("PIXI_PROJECT_ROOT"))
        child_env = _create_isaac_sim_environment()
        if sys.platform != "win32" and not is_pixi_environment:
            child_env["ROS_DISTRO"] = args.ros_distro

        if args.use_internal_libs:
            if sys.platform == "win32":
                print("ERROR: use_internal_libs is not supported on Windows.", file=sys.stderr)
                sys.exit(1)
            else:
                internal_lib_path = f"{filepath_root}/exts/isaacsim.ros2.core/{args.ros_distro}/lib"
                current_ld_path = child_env.get("LD_LIBRARY_PATH", "")
                child_env["LD_LIBRARY_PATH"] = (
                    f"{internal_lib_path}:{current_ld_path}" if current_ld_path else internal_lib_path
                )
                specific_path_to_remove = f"/opt/ros/{args.ros_distro}"
                version_to_remove = "jazzy" if args.ros_distro == "humble" else "humble"
                update_env_vars(version_to_remove, specific_path_to_remove, "LD_LIBRARY_PATH", child_env)
                update_env_vars(version_to_remove, specific_path_to_remove, "PYTHONPATH", child_env)
                update_env_vars(version_to_remove, specific_path_to_remove, "PATH", child_env)

        # Apply path exclusions AFTER all other modifications
        if args.exclude_install_path:
            exclude_paths_from_env(args.exclude_install_path, "LD_LIBRARY_PATH", child_env)
            exclude_paths_from_env(args.exclude_install_path, "PYTHONPATH", child_env)
            exclude_paths_from_env(args.exclude_install_path, "PATH", child_env)

        if args.ros_installation_path:
            # If a custom ros installation path is provided (can be comma-separated list)
            if sys.platform == "win32":
                print("ERROR: ros_installation_path is not supported on Windows.", file=sys.stderr)
                sys.exit(1)
            else:
                # Split by comma to handle multiple paths
                ros_paths = [path.strip() for path in args.ros_installation_path.split(",") if path.strip()]

                for ros_path in ros_paths:
                    # Check if it's a setup.bash file or a directory
                    if ros_path.endswith("setup.bash") or "setup.bash" in ros_path:
                        # It's a ROS installation setup file
                        source_cmd = f"source {shlex.quote(ros_path)} && env"
                        result = subprocess.run(
                            ["bash", "-c", source_cmd], capture_output=True, text=True, check=True, env=child_env
                        )

                        # Parse and apply environment variables
                        for line in result.stdout.splitlines():
                            if "=" in line:
                                key, value = line.split("=", 1)
                                if key in ["LD_LIBRARY_PATH", "PYTHONPATH", "PATH", "ROS_DISTRO"]:
                                    child_env[key] = value
                    else:
                        # It's a workspace install directory - add to environment variables
                        install_path = ros_path.rstrip("/")

                        # Add to LD_LIBRARY_PATH
                        current_ld_path = child_env.get("LD_LIBRARY_PATH", "")
                        if current_ld_path:
                            child_env["LD_LIBRARY_PATH"] = f"{install_path}/lib:{current_ld_path}"
                        else:
                            child_env["LD_LIBRARY_PATH"] = f"{install_path}/lib"

                        # Add to PYTHONPATH
                        current_python_path = child_env.get("PYTHONPATH", "")
                        if current_python_path:
                            child_env["PYTHONPATH"] = f"{install_path}/lib/python3/dist-packages:{current_python_path}"
                        else:
                            child_env["PYTHONPATH"] = f"{install_path}/lib/python3/dist-packages"

        # Only override RMW_IMPLEMENTATION when dds_type is explicitly set. Otherwise,
        # let the Isaac Sim launcher select its bundled default.
        if args.dds_type:
            dds_to_rmw = {"fastdds": "rmw_fastrtps_cpp", "cyclonedds": "rmw_cyclonedds_cpp", "zenoh": "rmw_zenoh_cpp"}
            if args.dds_type not in dds_to_rmw:
                print(
                    f"ERROR: Unsupported dds_type '{args.dds_type}'. Use one of: {', '.join(dds_to_rmw)}.",
                    file=sys.stderr,
                )
                sys.exit(1)
            child_env["RMW_IMPLEMENTATION"] = dds_to_rmw[args.dds_type]
        python_script = _resolve_python_script_path(args.python_script)

        popen_kwargs = {"shell": True, "env": child_env}
        if sys.platform == "win32":
            popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            popen_kwargs["start_new_session"] = True

        if args.standalone != "":
            if python_script:
                print(
                    "ERROR: python_script is only supported when standalone is empty. "
                    "Use standalone to run a standalone Isaac Sim Python workflow.",
                    file=sys.stderr,
                )
                sys.exit(1)

            executable_path = os.path.join(filepath_root, "python.sh" if sys.platform != "win32" else "python.bat")
            if sys.platform == "win32":
                proc = subprocess.Popen(f'"{executable_path}" {args.standalone}', **popen_kwargs)
            else:
                proc = subprocess.Popen(f"{executable_path} {args.standalone}", **popen_kwargs)
            subprocesses.append(proc.pid)
        else:
            # Default command
            if sys.platform == "win32":
                executable_command = f'"{os.path.join(filepath_root, "isaac-sim.bat")}" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'
            else:
                executable_command = f'{os.path.join(filepath_root, "isaac-sim.sh")} --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'

            if args.headless == "webrtc":
                if sys.platform == "win32":
                    executable_command = f'"{os.path.join(filepath_root, "isaac-sim.streaming.bat")}" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'
                else:
                    executable_command = f'{os.path.join(filepath_root, "isaac-sim.streaming.sh")} --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge'

            if args.custom_args != "":
                executable_command += f" {args.custom_args}"

            if args.gui != "" or python_script:
                scripts_dir = os.path.join(get_package_share_directory("isaacsim_bringup"), "scripts")
                startup_command_args = [os.path.join(scripts_dir, "open_isaacsim_stage.py")]
                if args.gui != "":
                    startup_command_args.extend(["--path", args.gui])
                if args.play_sim_on_start:
                    startup_command_args.append("--start-on-play")
                if python_script:
                    startup_command_args.extend(["--python-script", python_script])

                startup_command = _build_exec_command(startup_command_args)
                executable_command += f" --exec {_quote_command_argument(startup_command)}"

            proc = subprocess.Popen(executable_command, **popen_kwargs)
            subprocesses.append(proc.pid)


def main(args=None):
    rclpy.init(args=args)
    isaac_sim_launcher_node = IsaacSimLauncherNode()
    rclpy.spin(isaac_sim_launcher_node)
    # Ensure all subprocesses are terminated before exiting
    isaac_sim_shutdown()
    isaac_sim_launcher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
