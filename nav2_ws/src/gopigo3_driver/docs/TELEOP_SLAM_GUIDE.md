# Teleop SLAM Guide - Manual Teleoperation Mapping

This guide explains how to create and save a map using SLAM while controlling the GoPiGo3 robot with a keyboard.

---

## Overview

**Purpose**: Create an environment map by manually controlling the robot with a keyboard.

**Time Required**: 5-15 minutes depending on room size

**Requirements**:
- GoPiGo3 robot (powered ON)
- SSH connection or monitor/keyboard
- Space to map (room, hallway, etc.)

---

## Step-by-Step Procedure

### Step 0: Power On the Robot

1. Verify GoPiGo3 battery connection
2. Power ON the Raspberry Pi
3. Wait approximately 1 minute for boot completion
4. SSH connection:
   ```bash
   ssh ubuntu@<robot_IP_address>
   # Example: ssh ubuntu@192.168.1.100
   ```

---

### Step 1: Set Robot Starting Position

- Place the robot in the center of the room or a good starting position
- Ensure at least 30cm of clear space in front of the robot
- **This position becomes the map origin (0,0)**

---

### Step 2: Run Basic Driver (Terminal 1)

```bash
# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

# Run robot driver + sensors
ros2 launch gopigo3_driver gopigo3_bringup.launch.py
```

**Verify normal output**:
```
[INFO] [robot_state_publisher]: got segment base_footprint
[INFO] [gopigo3_driver]: GoPiGo3 driver started
[INFO] [bno055_imu]: BNO055 IMU initialized
[INFO] [rplidar_node]: RPLIDAR running
[INFO] [ekf_filter_node]: Starting...
```

**Wait 2-3 seconds** before proceeding to next step

---

### Step 3: Run SLAM (Terminal 2)

Open a new SSH session:
```bash
ssh ubuntu@<robot_IP_address>
```

Run SLAM:
```bash
# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

ros2 launch gopigo3_driver slam.launch.py
```

**Verify normal output**:
```
[INFO] [slam_toolbox]: Waiting for scan...
[INFO] [slam_toolbox]: Got scan, starting SLAM
```

---

### Step 4: Run Teleop (Keyboard Control) (Terminal 3)

Open a new SSH session:
```bash
ssh ubuntu@<robot_IP_address>
```

Start keyboard control:
```bash
# ROS2 environment setup (choose based on your shell)
source ~/nav2_ws/install/setup.zsh   # for zsh
source ~/nav2_ws/install/setup.bash  # for bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Control Keys**:
```
   u    i    o
   j    k    l
   m    ,    .

i : Forward
, : Backward
j : Turn left
l : Turn right
k : Stop
u : Forward + Turn left
o : Forward + Turn right
q/z : Increase/Decrease speed
```

**Recommended Speed Settings**:
- Adjust speed with `q` or `z`
- Linear velocity: 0.1 ~ 0.15 m/s
- Angular velocity: 0.3 ~ 0.5 rad/s

---

### Step 5: Perform Mapping

**Mapping Tips**:

1. **Move slowly**: Fast movement causes SLAM blur
2. **Follow walls**: Maintain 30-50cm distance from walls
3. **Visit all areas**: Cover all corners of the room
4. **Loop Closure**: Return to starting point (improves accuracy)
5. **Rotate in place**: Stop before rotating slowly

**Example Mapping Sequence**:
```
1. From start point, go to the right wall
2. Follow walls clockwise for one loop
3. Go around furniture in the center
4. Return to starting point
```

---

### Step 6: Real-time Map Verification (Optional)

**Method A: Foxglove Studio** (Recommended)
1. Run Foxglove Studio on PC
2. Connect to `ws://<robot_IP>:8765`
3. Add Map panel to visualize `/map` topic

**Method B: Check Topic in Terminal**
```bash
# Terminal 4
ros2 topic echo /map --once | head -20
```

---

### Step 7: Save Map

Save the map when mapping is complete.

**Method A: slam_toolbox Service (Recommended)**
```bash
# Terminal 4
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/my_room'}"
```

This command creates two files:
- `my_room.posegraph` - Pose graph
- `my_room.data` - Map data

**Method B: nav2_map_server (Standard Map Format)**
```bash
# Create save directory
mkdir -p ~/maps

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```

This command creates two files:
- `my_room.pgm` - Map image
- `my_room.yaml` - Map metadata

---

### Step 8: Shutdown

Press `Ctrl+C` in each terminal to shutdown:

1. Terminal 3: Stop teleop
2. Terminal 2: Stop SLAM
3. Terminal 1: Stop driver

---

## Verify Saved Map

```bash
# Check map files
ls -la ~/maps/

# Check map YAML contents
cat ~/maps/my_room.yaml
```

Expected output:
```yaml
image: my_room.pgm
resolution: 0.050000
origin: [-5.000000, -5.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## Map Quality Checklist

Verify the saved map quality:

- [ ] Walls are thin and straight (thick or double lines = bad)
- [ ] All rooms/areas are included in the map
- [ ] Start and end points match (Loop Closure success)
- [ ] Furniture and obstacles are shown on map
- [ ] Gray (unknown) areas are only outside walls

---

## Troubleshooting

### Problem: Map is distorted (drift)
- **Cause**: Moving or rotating too fast
- **Solution**: Move more slowly, perform Loop Closure

### Problem: Walls appear as double lines
- **Cause**: Odometry drift
- **Solution**: Return to starting point for Loop Closure

### Problem: Empty spaces in map
- **Cause**: Area was not visited
- **Solution**: Move near empty areas to scan

### Problem: SLAM doesn't start
- **Cause**: No LiDAR data
- **Solution**:
  ```bash
  ros2 topic echo /scan --once
  # If no data, check LiDAR connection
  ```

---

## Quick Reference Commands

> **Note**: Environment setup required in each terminal first
> - zsh: `source ~/nav2_ws/install/setup.zsh`
> - bash: `source ~/nav2_ws/install/setup.bash`

```bash
# Terminal 1: Robot driver
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# Terminal 2: SLAM
ros2 launch gopigo3_driver slam.launch.py

# Terminal 3: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Save map (Method B)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```
