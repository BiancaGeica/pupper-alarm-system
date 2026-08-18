# Thermal Map Node
ROS 2 node for the Pupper alarm system. Combines temperature data with the robot's position (from SLAM) to build a thermal map and publish alerts when dangerous temperatures are detected.


## Overview
This node subscribes to temperature readings on `/temperature_data` and uses the TF (Transform Frames) system to determine the robot's current position on the map. It saves `(x, y, temperature)` data points and publishes an alert on the `/alarm_event` ROS 2 topic when the temperature exceeds a configurable threshold, allowing the Telegram bot (`alarm_bot_node.py`) to notify the rescue team.

## 1. Prerequisites & Installation

### **Dependencies:**
1. **ROS 2 Jazzy** — install from the [official ROS 2 documentation](https://docs.ros.org/en/jazzy/Installation.html), or use the provided Dockerfile.
2. **SLAM system** — the node reads the robot's position from TF, which requires a running SLAM node (use `Proiect_Autonomous_Systems_Bootcamp` repository).
3. **Temperature bridge** — a bridge node that reads the MLX90614 sensor via USB Serial and publishes on `/temperature_data`.

### **Using Docker:**
1. Build the Docker image:
   ```bash
   docker build -t pupper-alarm .
   ```
2. Run the container:
   ```bash
   docker run -it pupper-alarm
   ```
3. Inside the container:
   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 thermal_map_node.py
   ```

## 3. Testing Without Hardware

1. **Terminal 1** — publish a fake TF transform (simulates the robot at position 0, 0):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
   ```
2. **Terminal 2** — run the thermal map node:
   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 thermal_map_node.py
   ```
3. **Terminal 3** — publish a fake temperature reading:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic pub /temperature_data std_msgs/String "data: '72'" --once
   ```
4. If Terminal 2 shows `Temp: 72.0°C la pozitia: (0.00, 0.00)`, the node is working correctly.
