# PhoneIMU

React Native (Expo) app to stream IMU data to a ROS2 WebSocket server.

## Overview

This application runs on Android and iOS devices using Expo. It accesses the device's accelerometer, gyroscope, and magnetometer (orientation) and streams the data via WebSockets to a companion ROS2 node.

**Features:**
- Streams **Accelerometer**, **Gyroscope**, and **Orientation** (DeviceMotion).
- Configurable sending rate (default 100Hz).
- Connects via WebSocket (TCP) to the ROS2 node.
- Visual logs for connection status and errors.

## Prerequisites

- **Node.js** (LTS recommended)
- **Expo Go** app installed on your mobile device (available on Play Store / App Store).

## Installation

1. Navigate to the directory:
   ```bash
   cd PhoneIMU
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

## Usage

1. Start the development server:
   ```bash
   npx expo start
   ```

2. **Connect your phone**:
   - Ensure your phone and computer are on the **same Wi-Fi network**.
   - Scan the QR code displayed in the terminal using the **Expo Go** app.

3. **Stream Data**:
   - Enter the **IP Address** of the computer running the ROS2 node (e.g., `192.168.1.X`).
   - Press **Connect**.
   - Toggle individual sensors to start/stop streaming specific data.

## Configuration

- **Frequency**: The default sending rate is **100Hz**. This can be modified in `App.tsx` by changing the `FREQUENCY_HZ` constant.
