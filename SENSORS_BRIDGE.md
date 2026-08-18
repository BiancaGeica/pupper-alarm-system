# Sensors Bridge
ROS 2 bridge node for the Pupper alarm system. Reads gas level data (MQ-5) and temperature data (MLX90614) from the Arduino Nano module via USB Serial.


## Overview
This script acts as a bridge between the Arduino Nano sensors and the ROS 2 environment. It continuously reads the USB Serial stream and routes the data:
- `GAS_LEVEL:` strings are checked against a threshold. If dangerous, an alert is published on the `/alarm_event` topic (for Telegram).
- `TEMP_OBJ:` strings are published directly to the `/temperature_data` topic (for the thermal map).

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
   python3 sensors_bridge.py
   ```

## 2. Testing Without Hardware

To verify the bridge logic without the physical Arduino Nano, you can simulate the USB serial connection using a virtual port named "socat".

1. **Terminal 1**: Start the container, install socat, and create the virtual serial cable:
   ```bash
   docker run -it --name bridge_test -v "$HOME/Desktop/pupper-alarm-system":/alarm pupper-alarm bash
   apt-get update && apt-get install -y socat
   socat PTY,link=/tmp/ttyFAKE_ARDUINO,raw,echo=0 PTY,link=/tmp/ttyFAKE_BRIDGE,raw,echo=0
   ```

   (Keep this terminal open, the socat process must keep running)

2. **Terminal 2**: Start the Python bridge node inside the container (make sure to temporarily change `SERIAL_PORT = '/tmp/ttyFAKE_BRIDGE'` in the script first, is allready there, just make sure to uncomment it)
   ```bash
   docker exec -it bridge_test bash
   source /opt/ros/jazzy/setup.bash
   pip3 install pyserial --break-system-packages
   python3 /alarm/sensors_bridge.py
   ```

3. **Terminal 3**: Listen for gas alarms:
   ```bash
   docker exec -it bridge_test bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic echo /alarm_event
   ```

4. **Terminal 4**: Listen for temperature data:
   ```bash
   docker exec -it bridge_test bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic echo /temperature_data
   ```

5. **Terminal 5**: Simulate the Arduino by writing fake data into the virtual cable:
   ```bash
   docker exec -it bridge_test bash
   echo "TEMP_OBJ:74.5" > /tmp/ttyFAKE_ARDUINO
   echo "GAS_LEVEL:400" > /tmp/ttyFAKE_ARDUINO
   ```

   You should immediately see the temperature output in Terminal 4 and the gas alarm in Terminal 3.

**IMPORTANT:** Please change change back SERIAL_PORT back to /dev/ttyUSB0 in sensors_bridge.py after testing!



## 4. TODO

*   Calibrate the gas threshold value based on real MQ-5 sensor readings
*   Verify the serial port on the Pupper robot to match the one in "SERIAL_PORT"
*   Enable the 2-minute warm-up delay for MQ-5 in production
