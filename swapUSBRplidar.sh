#!/bin/bash
if grep -q "ttyUSB0"  ~/catkin_ws/src/rplidar_ros/launch/rplidar.launch; then
	sed -i 's/ttyUSB0/ttyUSB1/g' ~/catkin_ws/src/rplidar_ros/launch/rplidar.launch
else
	sed -i 's/ttyUSB1/ttyUSB0/g' ~/catkin_ws/src/rplidar_ros/launch/rplidar.launch
fi
