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
ros2 launch isaac_ros2_control_demo ur10_in_process.launch.xml
```

RViz opens; Plan & Execute in the MotionPlanning panel drives the UR10 in
Isaac Sim. The launch activates both controllers after the ControllerManager
becomes available.

By default, the launch waits up to 60 seconds for Isaac Sim's transient-local
`/robot_description` message. For offline testing or a file-based integration,
pass `robot_description_file:=/absolute/path/to/robot.urdf`; use
`robot_description_timeout:=SECONDS` to change the topic wait timeout.

## vs `isaac_moveit`

- `isaac_moveit`: topic-based fake hardware, separate `ros2_control_node`.
- This package: real `IsaacSimSystem` hardware plugin loaded by an in-process
  ControllerManager. No external `ros2_control_node`; tighter physics-rate
  sync; matches real-robot bringup.
