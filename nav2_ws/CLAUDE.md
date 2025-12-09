# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 Humble workspace for the NeuronBot2 robot platform. The workspace contains packages for robot bringup, navigation, SLAM, Gazebo simulation, and hardware interfaces (RPLidar, serial communication).

## Build System

This is a ROS 2 workspace using colcon as the build system.

### Building the workspace

```bash
cd ~/gopigo_mount  # or wherever this workspace is located
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

### Installing dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Architecture

The workspace is organized into four main component repositories (defined in [neuronbot2_ros2.repos](neuronbot2_ros2.repos)):

### 1. NeuronBot2 Core (`src/neuronbot2/`)
Main robot packages:
- **neuronbot2_bringup**: Hardware bringup with C++ driver (`neuronbot2_driver`) for serial communication with robot base, robot state publisher, RPLidar interface, RealSense camera integration (optional), and EKF sensor fusion (optional)
- **neuronbot2_description**: URDF models with three variants (base, front camera, top camera) and robot meshes
- **neuronbot2_nav**: Navigation stack integration using Nav2 with separate launch files for localization, navigation, and combined bringup; includes pre-built maps (mememan, phenix, F9, training_room, wg)
- **neuronbot2_slam**: Three SLAM implementations (gmapping, slam_toolbox, cartographer)
- **neuronbot2_gazebo**: Simulation environment with two pre-built worlds (mememan_world, phenix_world)
- **neuronbot2_tools**: LED control and initialization scripts for USB device setup

### 2. RPLidar ROS (`src/rplidar_ros/`)
ROS 2 driver for RPLIDAR laser scanner with support for A1/A2/A3/S1 models.

### 3. Serial Communication (`src/serial/`)
Low-level serial communication library for robot base communication.

### 4. SLAM Gmapping (`src/slam_gmapping/`)
Grid-based SLAM implementation (includes both ROS wrapper and OpenSLAM gmapping library).

## Common Commands

### Real Robot

#### Bring up the robot hardware
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch neuronbot2_bringup bringup_launch.py
```

Optional parameters:
- `use_camera:=front` or `use_camera:=top` - Enable RealSense D435
- `use_ekf:=true` - Enable multi-sensor fusion

#### Teleop the robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Run SLAM
```bash
# Gmapping
ros2 launch neuronbot2_slam gmapping_launch.py open_rviz:=true

# Slam_toolbox
ros2 launch neuronbot2_slam slam_toolbox_launch.py open_rviz:=true

# Cartographer
ros2 launch neuronbot2_slam cartographer_launch.py open_rviz:=true
```

#### Save a map
```bash
ros2 run nav2_map_server map_saver_cli -f <map_dir>/<map_name>
```

#### Navigate with a map
```bash
ros2 launch neuronbot2_nav bringup_launch.py map:=<full_path_to_map.yaml> open_rviz:=true
```

### Simulation

#### Launch Gazebo simulation
```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# Mememan world
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model

# Phenix world
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=phenix_world.model
```

#### Run SLAM in simulation
Add `use_sim_time:=true` to any SLAM command:
```bash
ros2 launch neuronbot2_slam gmapping_launch.py open_rviz:=true use_sim_time:=true
```

#### Navigate in simulation
```bash
ros2 launch neuronbot2_nav bringup_launch.py \
  map:=$HOME/gopigo_mount/src/neuronbot2/neuronbot2_nav/map/mememan.yaml \
  open_rviz:=true \
  use_sim_time:=true
```

Alternative (separate terminals):
```bash
# Terminal 1: Localization
ros2 launch neuronbot2_nav localization_launch.py use_sim_time:=true

# Terminal 2: Navigation
ros2 launch neuronbot2_nav navigation_launch.py use_sim_time:=true

# Terminal 3: Visualization
ros2 launch neuronbot2_nav rviz_view_launch.py use_sim_time:=true
```

## Key Launch File Parameters

### Navigation (`neuronbot2_nav/launch/bringup_launch.py`)
- `map`: Path to map YAML file (default: mememan.yaml)
- `open_rviz`: true | false (default: false)
- `use_sim_time`: true | false (default: false) - must be true for simulation
- `slam`: true | false (default: false) - run SLAM instead of localization
- `params_file`: Path to Nav2 parameters file (default: neuronbot_params.yaml)

### Hardware Bringup (`neuronbot2_bringup/launch/bringup_launch.py`)
- `use_camera`: none | front | top (default: none)
- `use_ekf`: true | false (default: false)

## Important Notes

- When working with the real robot, run `neuronbot2_init.sh` once during first setup to configure USB device permissions
- The `use_sim_time` parameter must be set to `true` for all nodes when running in Gazebo simulation
- The neuronbot2_driver node publishes either `odom` or `raw_odom` depending on whether EKF is enabled
- Pre-built maps are located in `src/neuronbot2/neuronbot2_nav/map/`
- Close the Gazebo GUI (not the server) to reduce CPU usage during simulation
