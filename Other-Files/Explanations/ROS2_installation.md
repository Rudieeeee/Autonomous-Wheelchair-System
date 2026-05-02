things to install: 
WLS2 and ubuntu ((latest) version 24.4 for downloading jazzy)
ROS2 (Jazzy) 

in the wls environment download
- rVIZ2 for 3D visualization
- rqt to see node connections, plot sensor data, inspect topics

- gazebo (for simulating) 
- nav2 
- slam - toolbox

for getting serial data (as input devices are handled by windows we have to manually attach it)
- winget install usbipd (on windows powershell ran as administrator)
-  sudo apt install minicom screen picocom (in WSL)



After installing everything: 
before using anything connected to ros2 run source /opt/ros/jazzy/setup.bash
for rviz2 run: rviz2
for rqt run: rqt
for gazebo run: gz sim 



to connect a usb device to WSL (in windows powershell)
1. run usbipd list
2. read the busid of the desired device/devices
3.  for example for a busid of 1-1 run usbipd attach --wsl --busid 1-1
this has to be reran everytime you disconnect the device or restart the computer :D 
