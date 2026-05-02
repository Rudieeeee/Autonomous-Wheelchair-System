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
