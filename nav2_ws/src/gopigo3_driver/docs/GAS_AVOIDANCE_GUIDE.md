# Gas Avoidance Navigation System

Guide for the GoPiGo3 robot's gas detection-based obstacle avoidance system.

## Overview

This system detects hazardous gases using an MQ-2 gas sensor and sets the detected location as a virtual obstacle, causing Nav2 to automatically replan the path to avoid that area.

```
┌─────────────────────────────────────────────────────────────┐
│                    System Architecture                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌─────────────────┐    ┌───────────────┐   │
│  │ MQ-2     │───▶│ /gas_detection  │───▶│ Gas Obstacle  │   │
│  │ Sensor   │    │ (Float32)       │    │ Node          │   │
│  └──────────┘    └─────────────────┘    └───────┬───────┘   │
│                                                  │           │
│                                                  ▼           │
│  ┌──────────┐    ┌─────────────────┐    ┌───────────────┐   │
│  │ Saved    │───▶│ Nav2            │◀───│ /gas_obstacles│   │
│  │ Map      │    │ (Costmap)       │    │ (OccupancyGrid)   │
│  └──────────┘    └────────┬────────┘    └───────────────┘   │
│                           │                                  │
│                           ▼                                  │
│                  ┌─────────────────┐                        │
│                  │ Navigate to Exit │                        │
│                  │ (Avoid Gas Zone) │                        │
│                  └─────────────────┘                        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## File Structure

```
gopigo3_driver/
├── gopigo3_driver/
│   ├── gas_obstacle_node.py    # Gas detection → Obstacle conversion
│   └── gas_simulator_node.py   # Simulator for testing
├── config/
│   └── nav2_params.yaml        # gas_layer added
└── launch/
    └── navigation.launch.py    # Includes gas_obstacle node
```

## Quick Start

### 1. Build

```bash
cd ~/nav2_ws
colcon build --packages-select gopigo3_driver
source install/setup.bash
```

### 2. Run (Test Mode)

**Terminal 1: Start Navigation**
```bash
ros2 launch gopigo3_driver navigation.launch.py map:=/home/ubuntu/maps/my_room
```

**Terminal 2: Run Gas Simulator**
```bash
ros2 run gopigo3_driver gas_simulator
```

**Terminal 3: Connect with Foxglove**
- Set goal point in Foxglove
- Press 'g' key in gas simulator to trigger gas detection
- Nav2 automatically replans path

### 3. Simulator Keyboard Commands

| Key | Action |
|-----|--------|
| `g` | Trigger gas detection at current position (once) |
| `t` | Toggle continuous gas detection mode |
| `c` | Clear all gas obstacles |
| `q` | Quit |

## Sensor Integration

Publish to the following topics from your MQ-2 sensor code.

### Option 1: Float32 Concentration Value (Recommended)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MQ2SensorNode(Node):
    def __init__(self):
        super().__init__('mq2_sensor')
        self.publisher = self.create_publisher(Float32, '/gas_detection', 10)

    def publish_reading(self, concentration_ppm: float):
        msg = Float32()
        msg.data = concentration_ppm  # e.g., 350.0
        self.publisher.publish(msg)
```

### Option 2: Bool Detection Flag (Simple)

```python
from std_msgs.msg import Bool

# Must set use_bool_topic parameter to true
self.publisher = self.create_publisher(Bool, '/gas_detected', 10)

msg = Bool()
msg.data = True  # Gas detected
self.publisher.publish(msg)
```

## Parameter Configuration

### Gas Obstacle Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gas_threshold` | 300.0 | Danger threshold concentration (ppm) |
| `obstacle_radius` | 0.5 | Virtual obstacle radius (m) |
| `inflation_radius` | 0.3 | Additional safety distance (m) |
| `zone_timeout` | 300.0 | Obstacle retention time (seconds) |
| `use_bool_topic` | false | Whether to use Bool topic |

### Changing Parameters

**Method 1: Modify in Launch File**

In `navigation.launch.py`:
```python
gas_obstacle = Node(
    package='gopigo3_driver',
    executable='gas_obstacle',
    parameters=[{
        'gas_threshold': 200.0,  # Adjust for MQ-2 sensor
        'obstacle_radius': 0.8,
        ...
    }]
)
```

**Method 2: Command Line**
```bash
ros2 run gopigo3_driver gas_obstacle --ros-args \
    -p gas_threshold:=200.0 \
    -p obstacle_radius:=0.8
```

## Topics and Services

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/gas_detection` | Float32 | Input | Gas concentration (ppm) |
| `/gas_detected` | Bool | Input | Gas detection flag |
| `/gas_obstacles` | OccupancyGrid | Output | Virtual obstacle map |
| `/gas_zones` | MarkerArray | Output | Foxglove visualization |
| `/gas_alert` | Bool | Output | Gas warning notification |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/clear_gas_obstacles` | Empty | Clear all gas obstacles |

## Visualization in Foxglove

1. **Gas Zone Visualization**
   - Topic: `/gas_zones`
   - Display as MarkerArray in 3D panel
   - Yellow~red cylinders indicate gas concentration

2. **Costmap Verification**
   - Topic: `/global_costmap/costmap`
   - Gas zones appear as obstacles (red)

## Test Scenarios

### Scenario 1: Basic Avoidance Test

1. Start navigation
2. Set goal point in Foxglove
3. Press 'g' key in gas simulator while robot is moving
4. Verify robot avoids current position and takes new path

### Scenario 2: Multiple Gas Zones Test

1. Press 'g' key at multiple locations to create multiple gas zones
2. Verify robot finds path avoiding all zones

### Scenario 3: Real Sensor Integration

1. MQ-2 sensor node publishes to `/gas_detection` topic
2. Verify automatic avoidance when real gas is detected

## Troubleshooting

### Gas obstacles not appearing on costmap

```bash
# Check if topic is publishing
ros2 topic echo /gas_obstacles

# Check gas_obstacle node status
ros2 node info /gas_obstacle
```

### TF Error Occurs

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Verify base_footprint → map transform is available
ros2 run tf2_ros tf2_echo map base_footprint
```

### Path Not Replanning

Verify Nav2's costmap is properly subscribing to gas_layer:
```bash
ros2 param get /global_costmap/global_costmap plugins
# Should output: ["static_layer", "obstacle_layer", "gas_layer", "inflation_layer"]
```

## MQ-2 Sensor Integration Example (I2C)

Code to add to your MQ-2 sensor:

```python
#!/usr/bin/env python3
"""MQ-2 Gas Sensor ROS2 Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# I2C library (e.g., smbus2)
import smbus2

class MQ2SensorNode(Node):
    def __init__(self):
        super().__init__('mq2_sensor')

        # ROS2 Publisher
        self.gas_pub = self.create_publisher(Float32, '/gas_detection', 10)

        # I2C setup (modify address for your actual sensor)
        self.bus = smbus2.SMBus(1)
        self.sensor_addr = 0x48  # Example address

        # Read sensor at 10Hz
        self.timer = self.create_timer(0.1, self.read_sensor)

    def read_sensor(self):
        try:
            # Read gas concentration from I2C (modify for actual protocol)
            data = self.bus.read_i2c_block_data(self.sensor_addr, 0x00, 2)
            concentration = (data[0] << 8) | data[1]

            # Publish to ROS2 topic
            msg = Float32()
            msg.data = float(concentration)
            self.gas_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Sensor read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MQ2SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Future Improvements

- [ ] Pre-save exit locations
- [ ] Select optimal path among multiple exits
- [ ] Gas diffusion modeling (obstacle size changes over time)
- [ ] Automatic emergency mode entry (alarm integration)
