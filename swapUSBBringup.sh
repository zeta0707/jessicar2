if grep -q "ttyUSB0" src/jessicar2/jessicar2_bringup/launch/jessicar2_bringup.launch; then
	sed -i 's/port\" value=\"\/dev\/ttyUSB0/port\" value=\"\/dev\/ttyUSB1/g' src/jessicar2/jessicar2_bringup/launch/jessicar2_bringup.launch
else
	sed -i 's/port\" value=\"\/dev\/ttyUSB1/port\" value=\"\/dev\/ttyUSB0/g' src/jessicar2/jessicar2_bringup/launch/jessicar2_bringup.launch
fi

