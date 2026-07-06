# isaac_ros2_control_demo

Workspace half of the UR10 + MoveIt 2 in-process Controller Manager demo. The
in-Isaac-Sim half lives in
`omni_isaac_sim_ros2_control/source/standalone_examples/api/isaacsim.ros2.control/`.

## Prerequisites

Install the MoveIt 2 + RViz panels this demo depends on:

```bash
sudo apt install ros-jazzy-rviz-visual-tools
```

If you don't install it, the RViz config still loads but you'll see a
`PluginlibFactory: ... 'rviz_visual_tools/RvizVisualToolsGui' failed to load`
error. Cosmetic; MoveIt planning still works.

## Build

```bash
cd ~/IsaacSim-ros_workspaces/jazzy_ws
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

RViz opens; Plan & Execute in the MotionPlanning panel drives the UR10 in
Isaac Sim. The launch activates both controllers after the ControllerManager
becomes available.

## Self-test

To verify the whole pipeline without RViz (with Terminal 1 already running),
run the self-test launch. It starts `move_group`, activates the controllers,
plans and executes a motion, checks the UR10 reached the goal, and prints
`DEMO SELF-TEST: PASS` before shutting down:

```bash
ros2 launch isaac_ros2_control_demo ur10_in_process_test.launch.py
```

## vs `isaac_moveit`

- `isaac_moveit`: topic-based fake hardware, separate `ros2_control_node`.
- This package: real `IsaacSimSystem` hardware plugin loaded by an in-process
  ControllerManager. No external `ros2_control_node`; tighter physics-rate
  sync; matches real-robot bringup.
