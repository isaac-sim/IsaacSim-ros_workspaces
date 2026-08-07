# isaac_ros2_control_demo

Workspace half of the UR10 + MoveIt 2 in-process Controller Manager demo. The
in-Isaac-Sim half lives in
`omni_isaac_sim_ros2_control/source/standalone_examples/api/isaacsim.ros2.control/`.

## Build

```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
colcon build --packages-select isaac_ros2_control_demo
source install/setup.bash
```

## Run

Terminal 1 (Isaac Sim, in the omni_isaac_sim_ros2_control repo):

```bash
source _build/linux-x86_64/release/setup_ros_env.sh
./_build/linux-x86_64/release/python.sh \
    source/standalone_examples/api/isaacsim.ros2.control/ur10_ros2_control_demo.py
```

Terminal 2 (this workspace):

```bash
ros2 launch isaac_ros2_control_demo ur10_in_process.launch.py
```

The launch waits indefinitely for `/robot_description`, reports its status every
60 seconds, validates the message, and writes it to a temporary ROS 2 parameter
file before starting MoveIt. Press `Ctrl+C` to stop waiting.

RViz opens; Plan & Execute in the MotionPlanning panel drives the UR10 in
Isaac Sim. The launch activates both controllers after the ControllerManager
becomes available.

For offline testing or a file-based integration, pass
`robot_description_file:=/absolute/path/to/robot.urdf`. Use
`robot_description_log_interval:=SECONDS` to change the status interval.

## vs `isaac_moveit`

- `isaac_moveit`: topic-based fake hardware, separate `ros2_control_node`.
- This package: real `IsaacSimSystem` hardware plugin loaded by an in-process
  ControllerManager. No external `ros2_control_node`; tighter physics-rate
  sync; matches real-robot bringup.
