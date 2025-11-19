# phone_imu_bridge

ROS2 package that bridges phone IMU data to ROS2 topics via WebSockets.

## Overview

This package contains a ROS2 node that acts as a WebSocket server. It accepts JSON data payloads from the PhoneIMU app and publishes them as standard ROS2 IMU messages.

## Prerequisites

- **ROS2 Jazzy** (or compatible: Humble, Iron, Rolling)
- Python 3
- `python3-websockets` (usually installed via rosdep)

## Installation

1. Install dependencies using `rosdep`:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. Build the package:
   ```bash
   colcon build --packages-select phone_imu_bridge
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Run the tests:
   ```bash
   colcon test --packages-select phone_imu_bridge
   colcon test-result --verbose
   ```

## Usage

Run the server node:

```bash
ros2 run phone_imu_bridge imu_server_node
```

The server will start listening on `0.0.0.0:5000`.

## Nodes

### imu_server_node

Starts a WebSocket server on port `5000` and publishes received data.

#### Published Topics
- `/phone_imu` (`sensor_msgs/msg/Imu`)
  - **Frame ID**: `phone_imu_link`
  - **Rate**: Depends on the phone app's sending rate (default ~100Hz).

#### Parameters
*None currently implemented.*

**Future Improvements:**
- `port`: Configurable WebSocket port (default: 5000).
- `covariance`: Configurable covariance values (default: 0.01).
