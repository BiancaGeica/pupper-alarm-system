# Gas Sensor Bridge
ROS 2 bridge node for the Pupper alarm system. Reads gas level data from the Arduino Nano module via USB Serial and publishes alerts to the Telegram bot node.


## Overview
This script acts as a bridge between the Arduino Nano sensor module and the Telegram alerting system (alarm_bot_node.py). It reads the MQ-5 gas sensor values transmitted over USB Serial, compares them against a threshold, and publishes an alert message on the `/alarm_event` ROS 2 topic when dangerous gas levels are detected.

## 1. Prerequisites & Installation

### **Using Docker:**
1. Build the Docker image:
   ```bash
   docker build -t pupper-alarm .
   ```
2. Run the container with USB access:
   ```bash
   docker run -it --device=/dev/ttyUSB0 pupper-alarm
   ```
3. Inside the container:
   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 gas_sensor_bridge.py
   ```

## 2. Testing Without Hardware

You can verify that the ROS 2 messaging works without the Arduino Nano or the robot:

1. Start the Docker container:
   ```bash
   docker run -it pupper-alarm
   ```
2. **Terminal 1** — listen for alarm events:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic echo /alarm_event std_msgs/msg/String
   ```
3. **Terminal 2** — open a second shell in the same container and publish a test message:
   ```bash
   docker exec -it $(docker ps -q) bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic pub /alarm_event std_msgs/String "data: 'POSIBILA SCURGERE DE GAZE! Nivel gaz: 420'" --once
   ```
4. If the message appears in Terminal 1, the ROS 2 pipeline is working correctly.


## 4. TODO

*   Calibrate the gas threshold value based on real MQ-5 sensor readings
*   Verify the serial port on the Pupper robot to match the one in "SERIAL_PORT"
*   Enable the 2-minute warm-up delay for MQ-5 in production
