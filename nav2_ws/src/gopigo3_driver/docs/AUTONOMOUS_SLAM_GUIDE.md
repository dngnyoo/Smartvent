# Autonomous SLAM Guide - Autonomous Navigation-Based Mapping

This guide explains how the GoPiGo3 robot autonomously explores the environment and creates a map.

---

## Overview

**Purpose**: The robot autonomously finds unexplored areas (frontiers) and moves to them while creating a map.

**How It Works**:
1. **Frontier Exploration**: explore_lite finds and navigates to unexplored area boundaries
2. **Coverage Pattern**: When exploration stalls, zigzag pattern provides additional coverage
3. **Automatic Obstacle Avoidance**: Nav2 handles path planning and obstacle avoidance

**Time Required**: 10-30 minutes depending on room size (automatic)

**Requirements**:
- GoPiGo3 robot (powered ON)
- SSH connection
- Space to map (clear floor)

---

## Preparation

### Space Preparation
- [ ] Remove small obstacles from floor (cables, shoes, etc.)
- [ ] Keep doors open (to allow exploration space)
- [ ] Block gaps where robot could get stuck

### Robot Placement
- Place in center of room or open space
- Ensure at least 1m of clear space in front of robot
- **This position becomes the map origin (0,0) and return point**

---

## Step-by-Step Procedure

### Step 0: Power On the Robot

1. Verify GoPiGo3 battery connection
2. Power ON the Raspberry Pi
3. Wait approximately 1 minute for boot completion
4. SSH connection:
   ```bash
   ssh ubuntu@<robot_IP_address>
   ```

---

### Step 1: Run Autonomous SLAM (Terminal 1)

```bash
# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

# Run autonomous exploration + SLAM + Nav2 all at once
ros2 launch gopigo3_driver autonomous_slam.launch.py
```

**Startup Sequence** (automatic):
```
0s:   robot_state_publisher, bno055_imu start
2s:   gopigo3_driver starts (motors/encoders)
immediately: slam_toolbox, Nav2 servers start
10s:  explore_lite starts (frontier exploration)
15s:  coverage_explorer starts (backup exploration)
```

**Verify Normal Output**:
```
[INFO] [robot_state_publisher]: got segment base_footprint
[INFO] [gopigo3_driver]: GoPiGo3 driver started
[INFO] [bno055_imu]: BNO055 IMU initialized
[INFO] [rplidar_node]: RPLIDAR running
[INFO] [slam_toolbox]: Waiting for scan...
[INFO] [controller_server]: Creating controller server
[INFO] [planner_server]: Creating planner server
[INFO] [lifecycle_manager]: All nodes have been activated
[INFO] [explore_node]: Exploring...
[INFO] [coverage_explorer]: Coverage Explorer ready!
```

---

### Step 2: Monitor Exploration

The robot will automatically start exploring. Watch the logs:

**Normal Exploration Logs**:
```
[INFO] [explore_node]: Sending goal to frontier at (x, y)
[INFO] [controller_server]: Received new goal
[INFO] [bt_navigator]: Begin navigating...
```

**Coverage Mode Transition Logs**:
```
[INFO] [coverage_explorer]: No progress for 30.0s, switching to coverage mode
[INFO] [coverage_explorer]: Navigating to start position...
[INFO] [coverage_explorer]: Returned to start, generating coverage pattern
[INFO] [coverage_explorer]: Coverage waypoint 1/24: (0.50, 0.30)
```

---

### Step 3: Real-time Monitoring (Optional)

**Method A: Foxglove Studio** (from PC)
1. Launch Foxglove Studio
2. Connect to `ws://<robot_IP>:8765`
3. Add panels:
   - Map (`/map`)
   - LaserScan (`/scan`)
   - Path (`/plan`)
   - RobotModel

**Method B: Topic Monitoring** (Terminal 2)
```bash
ssh ubuntu@<robot_IP_address>

# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

# Check current exploration status
ros2 topic echo /explore/frontiers --once

# Check robot position
ros2 topic echo /odometry/filtered --once | grep -A3 position
```

---

### Step 4: Wait for Exploration Completion

When exploration is complete, the following messages appear:

**Frontier Exploration Complete**:
```
[INFO] [explore_node]: All frontiers traversed/tried out, stopping exploration
```

**Coverage Exploration Complete**:
```
[INFO] [coverage_explorer]: Coverage pass 1 completed
[INFO] [coverage_explorer]: Coverage pass 2 completed
[INFO] [coverage_explorer]: All coverage passes completed!
```

---

### Step 5: Save Map

Save the map when exploration is complete.

**Run in Terminal 2**:
```bash
ssh ubuntu@<robot_IP_address>

# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

# Create save directory
mkdir -p ~/maps

# Method A: slam_toolbox service (recommended)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/living_room'}"

# Method B: nav2_map_server (standard format)
ros2 run nav2_map_server map_saver_cli -f ~/maps/living_room
```

---

### Step 6: Shutdown

Press `Ctrl+C` in Terminal 1 to stop all nodes at once.

---

## Exploration Behavior Explanation

### Phase 1: Frontier Exploration (explore_lite)

```
┌─────────────────────────────────┐
│  ███░░░░░░░░░░░░░░░███          │
│  ███       ?        ███          │
│  ███    ┌───┐       ███          │
│         │ R │──→ Frontier        │
│         └───┘                    │
│  ███                ███          │
│  ███████████████████             │
└─────────────────────────────────┘
███ = Wall (obstacle)
░░░ = Unexplored area (Unknown)
?   = Frontier (exploration boundary)
R   = Robot
```

- Finds **boundaries (Frontiers)** between known and unknown areas
- Moves to nearest/most promising Frontier
- Scans new areas while moving
- Ends when all Frontiers are explored

### Phase 2: Coverage Pattern (coverage_explorer)

```
┌─────────────────────────────────┐
│  ███████████████████████████    │
│  ███  ←───────────────────      │
│  ███  ─────────────────→        │
│  ███  ←───────────────────      │
│  ███  ─────────────────→ R      │
│  ███████████████████████████    │
└─────────────────────────────────┘
```

- Activates after 30 seconds of no progress
- Returns to starting position
- Generates zigzag pattern within map boundaries
- Repeats twice to cover missed areas

---

## Parameter Adjustment

Adjustable in launch file if needed:

```python
# coverage_explorer parameters in autonomous_slam.launch.py
'coverage_spacing': 0.5,    # Zigzag spacing (m) - smaller = denser
'coverage_margin': 0.3,     # Distance from walls (m)
'coverage_passes': 2,       # Number of coverage repetitions
'frontier_timeout': 30.0,   # Transition wait time (seconds)
'goal_tolerance': 0.3       # Goal reach tolerance (m)
```

---

## Troubleshooting

### Problem: Robot doesn't move
```
[WARN] [controller_server]: No valid path found
```
**Solution**:
- Check if there's enough space around robot
- Check if LiDAR is detecting obstacles too close
- Move robot to more open space and restart

### Problem: Exploration stops too early
```
[INFO] [explore_node]: All frontiers traversed
```
(but gray areas remain on map)

**Solution**:
- coverage_explorer will automatically start zigzag pattern
- Wait 30 seconds for coverage mode transition

### Problem: Robot hitting obstacles
**Solution**:
```bash
# Increase inflation_radius in nav2_params.yaml
inflation_radius: 0.35  # Adjust to 0.4 or higher
```

### Problem: "Behavior Tree tick rate exceeded"
**Solution**: Pi4 CPU overload - already optimized, but if severe:
```bash
# Stop unnecessary processes
sudo systemctl stop bluetooth
```

### Problem: SPI Error
```
[ERROR] No SPI response
```
**Solution**:
```bash
# Retry - 2 second delay already applied
# If continues to fail:
sudo pigpiod
# Restart
```

---

## Quick Reference Commands

> **Note**: Environment setup required first
> - zsh: `source ~/nav2_ws/install/setup.zsh`
> - bash: `source ~/nav2_ws/install/setup.bash`

```bash
# Run everything at once
ros2 launch gopigo3_driver autonomous_slam.launch.py

# Save map (after exploration complete)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# Or slam_toolbox method
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/my_map'}"
```

---

## Exploration Completion Checklist

- [ ] Robot started moving automatically
- [ ] Map growing visible in Foxglove
- [ ] "All frontiers traversed" or "All coverage passes completed" message confirmed
- [ ] Map saved successfully
- [ ] Saved map files exist (`ls ~/maps/`)
