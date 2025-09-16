# Delivbot ROS 2 Package

Delivbot provides a complete ROS 2 workspace package for simulating and running the Delivbot mobile base. It bundles URDF, meshes, Gazebo simulation assets, and launch files for both simulation and real hardware bring-up.

## Highlights
- Unified `robot.launch.py` for real-robot bring-up (robot_state_publisher, lidar, RealSense, ODrive, joystick teleop, twist_mux).
- Modular launch files for each hardware subsystem: RealSense, Hesai lidar, and ODrive hoverboard driver.
- Gazebo simulation launch with twist mux, joystick teleop, ros2_control, and ROS ↔︎ Gazebo bridges.
- Ready-to-use URDF/Xacro, configuration, and RViz profiles for inspecting the robot.

## Package Layout

| Path | Description |
| ---- | ----------- |
| `config/` | Controller, bridge, teleop, and RViz configuration YAML files. |
| `launch/` | Gazebo, joystick, display, and hardware bring-up launch files (see below). |
| `meshes/` | Robot visuals and collision geometry. |
| `resource/` | Ament resource index markers. |
| `urdf/` | Xacro description of the Delivbot robot. |
| `worlds/` | Gazebo world files. |

## Build Instructions

```bash
# From the workspace root (e.g., ~/ros2_ws)
colcon build --packages-select delivbot --symlink-install
source install/setup.bash
```

## Simulation

Launch the full Gazebo simulation stack:

```bash
ros2 launch delivbot gazebo.launch.py
```

This starts Gazebo, spawns Delivbot, loads ros2_control controllers, the joystick teleop pipeline, and bridges available sensors to ROS topics.

For a URDF-only visualization in RViz:

```bash
ros2 launch delivbot display.launch.py
```

### Joystick usage in simulation
- Set joystick to D-Mode
- Enable/drive with left trigger (button 6) held.
- Turbo speed with right trigger (button 7).

## Real Robot Bring-Up

`robot.launch.py` runs the physical robot stack while keeping Gazebo assets untouched. It loads the URDF, publishes TFs, launches joystick teleop, and can selectively start the hardware drivers.

```bash
ros2 launch delivbot robot.launch.py \
  launch_lidar:=true \
  launch_realsense:=true \
  launch_odrive:=true \
  launch_joystick:=true \
  cmd_vel_topic:=/cmd_vel
```

Joystick commands flow through `twist_mux` so `/cmd_vel_joy` and navigation topics are arbitrated automatically before reaching the ODrive driver (default `/cmd_vel`).

### Joystick usage on hardware
- Set joystick to X-Mode
- Enable/drive with left shoulder (button 4) held.
- Turbo speed with right shoulder (button 5).

### Common Launch Arguments

| Argument | Default | Purpose |
| -------- | ------- | ------- |
| `robot_namespace` | `` (empty) | Apply a namespace to all nodes. |
| `model` | `<pkg>/urdf/delivbot.xacro` | Path to the robot description Xacro. |
| `use_sim_time` | `false` | Set to `true` only if you provide simulated time (e.g., from Gazebo). |
| `launch_lidar` | `true` | Toggle Hesai lidar driver. |
| `launch_realsense` | `true` | Toggle RealSense camera driver. |
| `launch_odrive` | `true` | Toggle ODrive hoverboard driver. |
| `launch_joystick` | `true` | Toggle joystick + teleop_twist pipeline. |
| `lidar_params_file` | `` (empty) | Optional YAML file for `hesai_ros_driver`. |
| `realsense_params_file` | `` (empty) | Optional YAML file for `realsense2_camera`. |
| `odrive_params_file` | `` (empty) | Optional YAML file for `odrive_hoverboard`. |
| `cmd_vel_topic` | `/cmd_vel` | Velocity command topic for the ODrive driver (also used by twist_mux output). |

### Subsystem Launch Files

| Launch file | Description | Notes |
| ----------- | ----------- | ----- |
| `launch/lidar.launch.py` | Starts `hesai_ros_driver_node`. | Supports namespace, params, and `use_sim_time`. |
| `launch/realsense.launch.py` | Starts `realsense2_camera_node`. | Provide a params YAML to customize camera streams. |
| `launch/odrive.launch.py` | Starts the `odrive_hoverboard` driver executable. | Exposes `cmd_vel_topic` and optional params YAML for tuning. |
| `launch/joystick.launch.py` | Starts `joy_node` and `teleop_twist_joy`. | Publishes `cmd_vel_joy`, which feeds the twist mux. |

Each subsystem launch file mirrors the `ros2 run` commands you would call manually, adding ergonomic launch arguments for consistent configuration.

## Dependencies

Ensure the following packages/drivers are available in your workspace or installed on the system:

- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`
- `hesai_ros_driver`
- `realsense2_camera`
- `odrive_hoverboard`
- `joy`, `teleop_twist_joy`
- `twist_mux`

Refer to each dependency’s documentation for hardware setup (e.g., USB permissions for RealSense cameras and ODrive).

## Suggested Workflow

1. **Simulation test:** Verify navigation stack interaction and teleoperation via `gazebo.launch.py`.
2. **Hardware dry-run:** Connect the robot and start `robot.launch.py`, disabling specific drivers if they are not attached yet (e.g., `launch_realsense:=false`).
3. **Parameter tuning:** Provide YAML configuration files through the launch arguments as you refine driver settings (e.g., wheel parameters for the ODrive driver).
4. **Monitoring:** Use `rqt_graph`, `rqt_tf_tree`, and RViz with `config/display.rviz` to visualize topics and transforms.

## Support & Contribution

For adjustments or new integrations, update the relevant launch file or configuration under `config/`, add documentation here, and rebuild with `colcon`. Contributions should include testing steps and documentation updates.
