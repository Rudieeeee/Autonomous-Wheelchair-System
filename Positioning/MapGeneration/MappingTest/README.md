# MappingTest README

## Purpose

`MappingTest` is the first package in the mapping, localization, and navigation pipeline. Its main purpose is to create a 2D map of the environment using recorded LiDAR and odometry data.

The package reads a CARMEN-style log file, converts the data into ROS2 topics, and uses SLAM Toolbox to generate a map.

The main output of this package is:

```text
my_map.pgm
my_map.yaml
```

These map files are later used by `LocalizationTest` and indirectly by `NavigationTest`.

---

## Position in the Full System

`MappingTest` is the first layer of the system:

```text
MappingTest
    ↓ creates map files
LocalizationTest
    ↓ estimates wheelchair pose inside the map
NavigationTest
    ↓ plans paths and publishes velocity commands
```

The mapping package does not depend on the localization or navigation packages. Instead, the other packages depend on the map created by `MappingTest`.

---

## Main Input

The main input is the recorded LiDAR and odometry file:

```text
/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/lidar.txt
```

This file contains `FLASER` entries. Each entry contains:

```text
LiDAR range readings
laser pose
odometry pose
```

The replay node reads this file and publishes the data as ROS2 topics.

---

## Main Outputs

During mapping, this package publishes:

```text
/scan
/odom
/tf
/map
```

### `/scan`

This topic contains the LiDAR scan data.

Message type:

```text
sensor_msgs/msg/LaserScan
```

### `/odom`

This topic contains odometry data.

Message type:

```text
nav_msgs/msg/Odometry
```

### `/tf`

This topic contains coordinate-frame transforms.

Important transforms are:

```text
odom -> base_footprint
base_footprint -> laser
```

### `/map`

This topic contains the generated occupancy grid map.

Message type:

```text
nav_msgs/msg/OccupancyGrid
```

---

## Saved Map Files

After mapping, the map is saved as:

```text
my_map.pgm
my_map.yaml
```

### `my_map.pgm`

This is the image representation of the map. It stores the occupied, free, and unknown areas.

### `my_map.yaml`

This is the metadata file for the map. It contains information such as:

```text
image path
resolution
origin
occupied threshold
free threshold
negate value
```

Example:

```yaml
image: my_map.pgm
resolution: 0.05
origin: [-47.795, -30.344, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

The `origin` and `resolution` are important because they define how image pixels relate to real-world map coordinates.

---

## Communication Flow Inside MappingTest

The mapping communication flow is:

```text
lidar.txt
   ↓
replay_carmen_log.py
   ↓ publishes
/scan, /odom, /tf
   ↓
slam_toolbox
   ↓ publishes
/map
   ↓
map_saver_cli
   ↓ saves
my_map.pgm + my_map.yaml
```

In words:

1. `replay_carmen_log.py` reads the recorded data file.
2. It publishes LiDAR data on `/scan`.
3. It publishes odometry data on `/odom`.
4. It publishes TF transforms on `/tf`.
5. SLAM Toolbox subscribes to these topics.
6. SLAM Toolbox generates the `/map` topic.
7. The generated map is saved as `.pgm` and `.yaml` files.

---

## Important Files in MappingTest

A typical package structure is:

```text
MappingTest/
├── package.xml
├── setup.py
├── resource/
│   └── mapping_test
├── mapping_test/
│   ├── __init__.py
│   └── replay_carmen_log.py
├── launch/
│   └── mapping_test.launch.py
├── config/
│   └── slam_toolbox.yaml
└── rviz/
    └── mapping.rviz
```

### `package.xml`

Defines the package name and dependencies.

Important dependencies include packages needed for:

```text
ROS2 Python package support
SLAM Toolbox
TF publishing
LiDAR and odometry messages
RViz visualization
```

### `setup.py`

Defines how the Python package is installed and which executable scripts are available.

The replay node is normally registered here so it can be run with:

```bash
ros2 run mapping_test replay_carmen_log
```

### `replay_carmen_log.py`

This is the main replay node.

It reads:

```text
lidar.txt
```

and publishes:

```text
/scan
/odom
/tf
```

This file is also important for the other packages because `LocalizationTest` uses the same replay node to replay scan and odometry data for AMCL.

### `mapping_test.launch.py`

This launch file starts the mapping process.

It can start:

```text
replay_carmen_log
slam_toolbox
RViz
```

This file is specific to mapping and should not start localization or navigation nodes.

### `slam_toolbox.yaml`

This file contains SLAM Toolbox parameters.

It affects how the map is built from LiDAR, odometry, and TF data.

### `mapping.rviz`

This RViz configuration is used to visualize mapping.

Useful RViz displays include:

```text
Map
LaserScan
Odometry
TF
```

---

## How MappingTest Connects to LocalizationTest

`MappingTest` creates the map files:

```text
my_map.pgm
my_map.yaml
```

`LocalizationTest` later loads these files with `map_server`.

The connection is:

```text
MappingTest
   ↓ saves
my_map.pgm + my_map.yaml
   ↓ used by
LocalizationTest map_server
```

`LocalizationTest` also uses the `replay_carmen_log.py` node from `MappingTest` to publish `/scan`, `/odom`, and `/tf` during localization testing.

So `LocalizationTest` depends on `MappingTest` in two ways:

```text
1. It uses the map created by MappingTest.
2. It uses the replay node from MappingTest.
```

---

## How MappingTest Connects to NavigationTest

`NavigationTest` does not directly use the mapping process, but it indirectly depends on the map created by `MappingTest`.

The connection is:

```text
MappingTest
   ↓ creates map
LocalizationTest
   ↓ loads map and publishes pose
NavigationTest
   ↓ uses map and pose for path planning
```

Nav2 needs the map because the global costmap and planner use it to plan paths through free space.

---

## Build Instructions

Go to the package folder:

```bash
cd /home/rudrh/Autonomous-Wheelchair-System/Positioning/MapGeneration/MappingTest
```

Source ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

Build the package:

```bash
colcon build \
  --packages-select mapping_test \
  --build-base build \
  --install-base install \
  --symlink-install
```

Source the package:

```bash
source install/setup.bash
```

---

## Run Instructions

Run mapping:

```bash
ros2 launch mapping_test mapping_test.launch.py
```

Useful checks:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## Summary

`MappingTest` is responsible for creating the map. It converts recorded data into ROS2 topics, runs SLAM Toolbox, and saves the map files.

Its most important outputs are:

```text
my_map.pgm
my_map.yaml
```

These files are required for localization and navigation.
