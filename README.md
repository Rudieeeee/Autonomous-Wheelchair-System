# Autonomous Wheelchair System

## Overview

This project develops an autonomous driving system for a Dietz Sango powered wheelchair. The system is intended for low-speed navigation in a known indoor environment. The user selects a destination by voice or through an on-screen map fallback, after which the wheelchair localizes itself, plans a route, monitors obstacles, and sends drive commands to the wheelchair controller.

The prototype combines user input, mapping, localization, obstacle detection, path planning, data communication, power distribution, and wheelchair control into one modular system. It uses two RPLIDAR C1 LiDARs, an Adafruit BNO085 IMU, Hall encoder sensors, VL53L7CX time-of-flight sensors, ESP32-S3 sensor nodes, an Arduino Giga, and a ROS2/Nav2-based software stack.

## Key Features

- Destination selection by voice with an on-screen map fallback
- Autonomous low-speed navigation in a known indoor map
- LiDAR-based map generation using SLAM Toolbox
- AMCL localization in a saved indoor map
- ROS2/Nav2 path planning and path following
- Real-time obstacle detection using time-of-flight sensors
- Wheelchair command interface through microcontroller and CAN-based control
- Modular architecture for future testing and extension

## System Architecture

The system is divided into five main subsystems. The **user input subsystem** handles voice commands, map-based destination selection, confirmation, feedback, and emergency stop input. The **positioning subsystem** generates reusable indoor maps and estimates the wheelchair pose in the map. The **obstacle detection subsystem** detects static and dynamic obstacles around the wheelchair. The **navigation subsystem** plans and follows a route using ROS2/Nav2. The **data and power subsystem** handles communication between the sensors, laptop, microcontrollers, and wheelchair, and supplies power to the added hardware.

## Setup

This project contains both Python-based user input software and ROS2-based mapping, localization, and navigation software. The voice interface runs on Windows, while the ROS2 mapping, localization, and navigation stack is intended to run on Linux.

---

## User Input Setup

### Requirements

- Python 3.10 or 3.11
- NumPy 1.x `< 2.0`
- Vosk speech model
- DeepFilterNet
- PyTorch CPU build

### Vosk Speech Model

Download `vosk-model-en-us-0.22` from:

```text
https://alphacephei.com/vosk/models
```

Extract it into the `Models` folder inside the repository. The structure should look like this:

```text
project/
└── UserInput/
    └── Voice control & GUI/
        ├── main.py
        └── Models/
            └── vosk-model-en-us-0.22/
```

### Python Virtual Environment

It is recommended to run the user input software inside a virtual environment:

```bash
python -m venv venv
venv\Scripts\activate           # Windows Command Prompt
./venv/Scripts/Activate.ps1     # Windows PowerShell
source venv/bin/activate        # Linux / macOS
```

### Install Python Dependencies

Install PyTorch CPU build first:

```bash
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cpu
```

Then install the remaining dependencies:

```bash
pip install -r requirements.txt
pip install deepfilternet
```

Everything else the user input system needs is listed in `requirements.txt`.

---

## ROS2 Setup for Mapping, Localization, and Navigation

The mapping, localization, and navigation software is intended to run on Ubuntu with ROS2 Jazzy. The commands below assume Ubuntu 24.04 with ROS2 Jazzy.

### 1. Install ROS2 Jazzy

Follow the official ROS2 Jazzy installation guide, or use the summary below:

```bash
sudo apt update
sudo apt install software-properties-common curl gnupg lsb-release -y

sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop -y
```

Add ROS2 to the shell startup file:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install ROS2 Build Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip git -y
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` says it was already initialized, continue with:

```bash
rosdep update
```

### 3. Install Navigation, Mapping, and Sensor Packages

Install the main ROS2 packages used for mapping, localization, navigation, RViz, rosbridge, and sensor processing:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf-transformations \
  ros-jazzy-robot-localization \
  ros-jazzy-laser-geometry \
  ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-rosbridge-server \
  ros-jazzy-teleop-twist-keyboard \
  python3-serial
```

### 4. Clone the Repository

```bash
cd ~
git clone https://github.com/Rudieeeee/Autonomous-Wheelchair-System.git
cd ~/Autonomous-Wheelchair-System
```

If the repository already exists:

```bash
cd ~/Autonomous-Wheelchair-System
git pull
```

### 5. Install External ROS2 Packages Used by the System

The system uses the Slamtec LiDAR driver and the laser scan merger package. These should be inside the ROS2 workspace `src` folder.

```bash
cd ~/Autonomous-Wheelchair-System/src

git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/mich1342/ros2_laser_scan_merger.git
```

If one of these folders already exists, update it instead:

```bash
cd ~/Autonomous-Wheelchair-System/src/sllidar_ros2
git pull

cd ~/Autonomous-Wheelchair-System/src/ros2_laser_scan_merger
git pull
```

### 6. Install Missing Dependencies

From the workspace root:

```bash
cd ~/Autonomous-Wheelchair-System

rosdep install --from-paths src --ignore-src -r -y
```

### 7. Build the Workspace

```bash
cd ~/Autonomous-Wheelchair-System

colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

To source the workspace automatically in every new terminal:

```bash
echo "source ~/Autonomous-Wheelchair-System/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Hardware Port Setup

The system uses USB devices for the LiDARs and Arduino. Give the user access to serial ports:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in after running this command.

Check connected USB devices:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Typical examples:

```text
/dev/ttyUSB0  -> left LiDAR
/dev/ttyUSB1  -> right LiDAR
/dev/ttyACM0  -> Arduino Giga
```

For navigation, the launch file can also use fixed device names:

```text
/dev/left_lidar
/dev/right_lidar
/dev/arduino_wheelchair
```

These names only work if udev rules have been created. If no udev rules are used, pass the normal `/dev/ttyUSB*` and `/dev/ttyACM*` ports as launch arguments.

---

## Running the System

### 1. Mapping

Mapping starts the sensor pipeline and SLAM Toolbox. The mapping launch file includes the LiDARs, static transforms, laser scan merger, pointcloud-to-laserscan node, Arduino sensor node, SLAM Toolbox, and RViz.

```bash
cd ~/Autonomous-Wheelchair-System
source install/setup.bash

ros2 launch map_generator mapping.launch2.py \
  left_lidar_port:=/dev/ttyUSB0 \
  right_lidar_port:=/dev/ttyUSB1 \
  arduino_port:=/dev/ttyACM0 \
  use_rviz:=true
```

To save a map manually:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/my_map \
  --ros-args -p map_subscribe_transient_local:=true
```

This creates:

```text
my_map.yaml
my_map.pgm
```

The mapping launch file also supports automatic repeated map saving:

```bash
ros2 launch map_generator mapping.launch2.py \
  left_lidar_port:=/dev/ttyUSB0 \
  right_lidar_port:=/dev/ttyUSB1 \
  arduino_port:=/dev/ttyACM0 \
  save_map:=/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/test_map \
  auto_save_map:=true \
  auto_save_period:=60.0 \
  use_rviz:=true
```

### 2. Localization

Localization starts the sensor pipeline, map server, AMCL, lifecycle manager, RViz, and a delayed global localization call.

```bash
cd ~/Autonomous-Wheelchair-System
source install/setup.bash

ros2 launch localization localization.launch.py \
  map:=/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/tellegen.yaml \
  left_lidar_port:=/dev/ttyUSB0 \
  right_lidar_port:=/dev/ttyUSB1 \
  arduino_port:=/dev/ttyACM0 \
  use_rviz:=true
```

If needed, set the starting pose in RViz using the **2D Pose Estimate** tool.

### 3. Navigation

Navigation starts localization first and then starts the Nav2 planner, controller, behavior tree navigator, behavior server, velocity smoother, safety limiter, and joystick command converter after a delay.

With fixed udev port names:

```bash
cd ~/Autonomous-Wheelchair-System
source install/setup.bash

ros2 launch navigation navigation.launch.py \
  map:=/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/ampere.yaml \
  left_lidar_port:=/dev/left_lidar \
  right_lidar_port:=/dev/right_lidar \
  arduino_port:=/dev/arduino_wheelchair \
  use_rviz:=true
```

Without udev rules, use the normal USB ports:

```bash
ros2 launch navigation navigation.launch.py \
  map:=/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/ampere.yaml \
  left_lidar_port:=/dev/ttyUSB0 \
  right_lidar_port:=/dev/ttyUSB1 \
  arduino_port:=/dev/ttyACM0 \
  use_rviz:=true
```

The navigation stack publishes velocity commands, limits them, smooths them, applies ToF safety filtering, and converts the safe velocity command into joystick commands for the wheelchair controller.

### 4. Rosbridge for User Input Connection

The Windows user input interface communicates with ROS2 through rosbridge.

Start rosbridge on the Linux/ROS2 side:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The user input interface can then publish destination goals and emergency stop messages through the WebSocket bridge.

---

## Important ROS2 Topics

Common topics used in the system include:

```text
/scan                  Merged LiDAR scan
/scan_left             Left LiDAR scan
/scan_right            Right LiDAR scan
/tof_scan              Time-of-flight obstacle scan
/odom                  Wheelchair odometry
/amcl_pose             Estimated wheelchair pose from AMCL
/map                   Occupancy grid map
/plan                  Planned navigation path
/cmd_vel_raw           Raw velocity command from Nav2 controller
/cmd_vel_limited       Limited velocity command
/cmd_vel               Smoothed velocity command
/cmd_vel_safe          Safety-filtered velocity command
/joystick_cmd          Joystick command sent to the wheelchair interface
/wheelchair/nav_goal   User-selected navigation goal
/wheelchair/status     Navigation status feedback
/wheelchair/estop      Emergency stop signal
```

## Reports and Documentation

- Literature Study: https://www.overleaf.com/read/pxfggznvhxvc#7b4c1a
- Program of Requirements (PoR): https://www.overleaf.com/read/cvwwdwxjmvfz#97bd66
- Meeting Notes: https://docs.google.com/document/d/1JzAj3k3fk30Rmm3bZLfRvXuysjHTRjCx-jTIksM33J0/edit?tab=t.0
- Gantt Chart: https://docs.google.com/spreadsheets/d/1LeIonz3t87s3dJxeFGJFFnhabDehzJCSFnnsxjkrOPM/edit?gid=1115838130#gid=1115838130
- Wheelchair Measurements: https://docs.google.com/spreadsheets/d/1bqGMkIiJ7xfXbivKcnknhoT9pYHNBYwShCx7t04MR1Q/edit?usp=sharing
- Plan B: https://www.overleaf.com/read/nyvbtqkbtnqg#3270e9
- Verification: https://www.overleaf.com/6864522186prfshsymjchx#0db520
- Design Report: https://www.overleaf.com/read/fdnncktpzdds#f19f40

## Team

BAP 2026 – Group nA6

- Ethan Croeze
- Rudrh Kapoor
- Ansh Kaushal
- Omar Shousha
- Guido Nuijt
- Dyorno Pavion

## License

This project is developed for academic purposes at TU Delft.
