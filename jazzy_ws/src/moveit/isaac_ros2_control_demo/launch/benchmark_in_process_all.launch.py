# SPDX-License-Identifier: Apache-2.0
"""All-in-one benchmark launch for in_process mode.

The benchmark script spawns controllers internally (same DDS context as the
in-process ControllerManager) so no external spawner nodes are needed.

Single terminal:
  ros2 launch isaac_ros2_control_demo benchmark_in_process_all.launch.py

Optional:
  ... install_path:=/path/to/_build/linux-x86_64/release
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BENCHMARK_SCRIPT = (
    "/home/aayoung/Pseudos/nvidia/isaac-sim/omni_isaac_sim_ros2_control"
    "/source/standalone_examples/testing/isaacsim.ros2.control/benchmark_ros2_control.py"
)
DEFAULT_INSTALL_PATH = (
    "/home/aayoung/Pseudos/nvidia/isaac-sim/omni_isaac_sim_ros2_control"
    "/_build/linux-x86_64/release"
)

# --wait 20 gives the background controller spawner ~17s to activate
# (3s startup delay + ~10s for two spawner calls).
STANDALONE_ARGS = f"{BENCHMARK_SCRIPT} --mode in_process --wait 35 --warmup 30 --steps 500"


def generate_launch_description() -> LaunchDescription:
    isaac_sim = Node(
        package="isaacsim_bringup",
        executable="run_isaacsim",
        name="isaacsim_bringup",
        output="screen",
        parameters=[{
            "install_path": LaunchConfiguration("install_path"),
            "standalone": STANDALONE_ARGS,
        }],
    )

    # Trajectory publisher starts after controllers are likely active.
    traj_publisher = TimerAction(period=25.0, actions=[Node(
        package="isaac_ros2_control_demo",
        executable="benchmark_trajectory_publisher",
        output="screen",
    )])

    # Shut down everything when Isaac Sim exits.
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=isaac_sim, on_exit=[Shutdown()])
    )

    return LaunchDescription([
        DeclareLaunchArgument("install_path", default_value=DEFAULT_INSTALL_PATH,
                              description="Isaac Sim _build release dir"),
        isaac_sim,
        traj_publisher,
        shutdown_on_exit,
    ])
