# Pixi-ROS Workspace

This ROS Humble workspace is configured to use [Pixi](https://pixi.sh) for dependency management.

## Getting Started

> **Warning: Windows path length limitation**
> Windows limits many file operations to paths of 260 characters. Clone this workspace into a short path, such as `C:\IsaacSim-ros_workspaces\humble_ws`. If `pixi run sim` or the build reports missing files, build errors, or `The filename or extension is too long`, move the repository to a shorter path.

### 1. Fetch git submodules

Several workspace packages are tracked as git submodules, including Greenwave Monitor
under `src/greenwave_monitor` and MoveIt-related packages under `src/moveit/`. After
cloning, you must initialize them or the workspace will fail to build with missing-package
errors:

```bash
git submodule update --init --recursive
```

If you cloned with `git clone --recurse-submodules`, this step is already done.

### 2. Install dependencies

The workspace declares its required Pixi compatibility in `pixi.toml`. If `pixi install`
reports that the workspace requires a newer Pixi version or reports an unsupported lock-file
version, update Pixi first:

```bash
pixi self-update
```

Then install the workspace dependencies:

```bash
pixi install
```

### 3. Build the workspace

```bash
pixi run build
```

### 4. Activate the environment

```bash
pixi shell
```

This starts a new shell with the ROS environment activated. You can also prefix any command
with `pixi run <command>` without entering the shell.

> **Note:** After the first `pixi run build`, pixi will automatically source
> `install/setup.(bash/bat)` on environment activation. This means all your built packages
> are available without any manual sourcing.

### 5. Start Isaac Sim

There is a `zenoh` and `sim` task you can run.

**Terminal 1:**

Start the zenoh server:

```bash
pixi run zenoh
```

**Terminal 2:**

Start the simulator:

```bash
pixi run sim
```

Now add the ROS 2 clock publisher from the sim and start the sim to test the ROS bridge:

- Click through: **Tools -> Robotics -> ROS 2 OmniGraphs -> Clock -> OK**
- Press the Play button in the simulator.

**Terminal 3:**

Check whether the topic became visible for the ROS CLI:

```bash
pixi run ros2 topic list
/clock
/parameter_events
/rosout
```

## Adding dependencies

When you add dependencies to your `package.xml` files, re-run `pixi ros init` to update `pixi.toml`:

```bash
pixi ros init --distro humble
```

To add a conda package directly:

```bash
pixi add <package-name>
```

To add a PyPI package:

```bash
pixi add --pypi <package-name>
```

## Unavailable packages

If `pixi.toml` contains commented-out lines marked `# NOT FOUND`, those packages could not be
resolved from the default channels. Options:

1. **Use a custom channel** — re-run init with `--channel` if the package lives elsewhere:
   ```bash
   pixi ros init --distro humble --channel https://prefix.dev/my-channel
   ```
2. **Add the channel manually** — `pixi project channel add <channel-url>`
3. **Check the package name** — verify spelling in your `package.xml`
4. **Install via PyPI** — `pixi add --pypi <package-name>`
5. **Contribute to RoboStack** — if the package is missing from the default ROS channel:
   - [ros-humble](https://github.com/RoboStack/ros-humble)
   - [ros-jazzy](https://github.com/RoboStack/ros-jazzy)
   - [ros-kilted](https://github.com/RoboStack/ros-kilted)

## Common issues

### Build fails

1. Make sure all dependencies are installed: `pixi install`
2. Clean and rebuild: `pixi run clean && pixi run build`

### `ros2` commands not found

Run commands through pixi: `pixi run <command>` or enter the shell with `pixi shell`.

### Active ROS environment conflict

If `pixi ros init` warns about an "Active ROS environment detected", you have a ROS installation
sourced in your shell. Remove or comment out any lines like the following from `~/.bashrc` or
`~/.zshrc` and restart your shell:

```bash
# source /opt/ros/humble/setup.bash
# source ~/ros_ws/install/setup.bash
```

Pixi manages the ROS environment automatically; no manual sourcing is needed.

## Learn more

- [Pixi documentation](https://pixi.sh)
- [RoboStack](https://robostack.github.io/)
- [ROS Humble documentation](https://docs.ros.org/en/humble/)
- [pixi-ros](https://github.com/prefix-dev/pixi-ros)
- [Isaac Sim ROS 2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
