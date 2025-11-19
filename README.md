# phone-imu-ros2

ROS2 Package and React Native App to stream IMU data from an Android or iOS phone to a ROS2 robot.

## Overview

This repository contains two main components:
1. **PhoneIMU**: A React Native (Expo) app that reads Accelerometer, Gyroscope, and Orientation data from a phone and streams it via WebSockets (see note below).
2. **phone_imu_bridge**: A ROS2 package with a node that acts as a WebSocket server, receiving the data and publishing it to a ROS topic.

## Compatibility

- **Mobile**: Works on **Android** and **iOS** thanks to React Native (Expo). Tested with NodeJS v24.5 (current LTS).
> [!NOTE]
> WebSockets (TCP) are used instead of UDP to ensure compatibility with Expo Go without needing native code changes. This should introduce a negligible latency overhead compared to raw UDP but allows for easy cross-platform deployment.
- **ROS2**: Tested on **ROS2 Jazzy**. Likely compatible with Humble/Iron/Rolling as it uses standard messages and Python client libraries.

## Installation as a submodule

To add this repository as a submodule to an existing ROS2 repository:

```bash
cd your-project
git submodule add https://github.com/b-Tomas/phone-imu-ros2.git src/phone-imu-ros2
git submodule update --init --recursive
```

Then build your workspace as usual:
```bash
colcon build --packages-select phone_imu_bridge
```

## Quick Start

### 1. Server side (ROS2)

**Prerequisites**:
- ROS2 Jazzy (or compatible)

The ROS2 node listens on port `5000` (TCP).

**Build**:

```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select phone_imu_bridge
source install/setup.bash
```

**Run**:

```bash
ros2 run phone_imu_bridge imu_server_node
```

### 2. Client side (Mobile)

**Prerequisites**:
- Node.js & npm
- Expo Go app on Android phone

**Dependency install**:
```bash
cd PhoneIMU
npm install
```

**Run**:
```bash
cd PhoneIMU
npx expo start
```
1. Scan the QR code with the [Expo Go app](https://expo.dev/go).
2. Enter the IP Address of the machine running the ROS2 node.
3. Press **Connect**.

## Data Format

The app sends JSON data:
```json
{
  "accel": {"x": 0.1, "y": 0.2, "z": 9.8},
  "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"alpha": 0, "beta": 0, "gamma": 0}
}
```

The ROS2 node publishes to `/phone_imu` (`sensor_msgs/msg/Imu`).
- **Frame ID**: `phone_imu_link`
- **Covariance**: Fixed diagonal values (0.01).
