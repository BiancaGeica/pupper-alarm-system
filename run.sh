#!/bin/bash

echo "=== Pregatire Mediu Docker pentru Pupper Alarm System ==="
echo "1. Se construieste imaginea..."
sudo docker build -t pupper-alarma .

echo "2. Se porneste dispecerul ROS 2..."
sudo docker run -it --rm --net=host -v $(pwd):/ws pupper-alarma bash -c "source /opt/ros/jazzy/setup.bash && python3 alarm_bot_node.py"