#!/bin/bash
if grep -q "ttyUSB0"  ~/catkin_ws/src/jessicar2/jessicar2_bringup/launch/arduino_bringup.launch; then
	sed -i 's/ttyUSB0/ttyUSB1/g' ~/catkin_ws/src/jessicar2/jessicar2_bringup/launch/arduino_bringup.launch
else
	sed -i 's/ttyUSB1/ttyUSB0/g' ~/catkin_ws/src/jessicar2/jessicar2_bringup/launch/arduino_bringup.launch
fi
