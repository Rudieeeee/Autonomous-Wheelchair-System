# LocalizationTest README

## Purpose

`LocalizationTest` is the second package in the mapping, localization, and navigation pipeline. Its main purpose is to estimate the wheelchair position inside a previously saved map.

It uses:

```text
map_server
AMCL
replayed LiDAR and odometry data
```

The package answers the question:

```text
Where is the wheelchair on the map?
```

---

## Position in the Full System

`LocalizationTest` sits between mapping and navigation:

```text
MappingTest
    ↓ creates my_map.pgm and my_map.yaml
LocalizationTest
    ↓ estimates wheelchair pose
NavigationTest
    ↓ plans a path and publishes velocity commands
```

`LocalizationTest` depends on `MappingTest` because it uses:

```text
the saved map created by MappingTest
the replay_carmen_log node from MappingTest
```

`NavigationTest` depends on `LocalizationTest` because navigation needs the estimated wheelchair pose.

---

## Main Inputs

### Saved Map

`LocalizationTest` uses the map created by `MappingTest`:

```text
/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/my_map.yaml
/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/Maps/my_map.pgm
```

The map is loaded by `map_server` and published on:

```text
/map
```

### Recorded Sensor Data

The package also uses the recorded LiDAR and odometry file:

```text
/home/rudrh/Autonomous-Wheelchair-System/Other-Files/GeneralData/lidar.txt
```

The replay node from `MappingTest` reads this file and publishes:

```text
/scan
/odom
/tf
```

---

## Main Outputs

`LocalizationTest` publishes:

```text
/map
/scan
/odom
/tf
/amcl_pose
/particle_cloud
```

### `/map`

Published by `map_server`.

Message type:

```text
nav_msgs/msg/OccupancyGrid
```

### `/scan`

Published by the replay node.

Message type:

```text
sensor_msgs/msg/LaserScan
```

### `/odom`

Published by the replay node.

Message type:

```text
nav_msgs/msg/Odometry
```

### `/amcl_pose`

Published by AMCL.

This is the estimated wheelchair pose in the map.

### `/particle_cloud`

Published by AMCL.

This shows the distribution of particles used by the particle filter.

### `/tf`

The most important transform published by AMCL is:

```text
map -> odom
```

The replay node publishes:

```text
odom -> base_footprint
base_footprint -> laser
```

Together, these transforms create the full localization chain:

```text
map
 └── odom
      └── base_footprint
           └── laser
```

---

## Communication Flow Inside LocalizationTest

The localization communication flow is:

```text
my_map.yaml + my_map.pgm
   ↓
map_server
   ↓ publishes
/map

lidar.txt
   ↓
replay_carmen_log node from MappingTest
   ↓ publishes
/scan, /odom, /tf

/map + /scan + /odom + /tf
   ↓
AMCL
   ↓ publishes
/amcl_pose
/particle_cloud
map -> odom transform
```

In words:

1. `map_server` loads the saved map.
2. The replay node publishes scan, odometry, and TF data.
3. AMCL compares the LiDAR scan against the saved map.
4. AMCL estimates where the wheelchair is.
5. AMCL publishes `/amcl_pose` and the `map -> odom` transform.

---

## Important Files in LocalizationTest

A typical package structure is:

```text
LocalizationTest/
├── package.xml
├── setup.py
├── resource/
│   └── localization_test
├── launch/
│   └── localization_test.launch.py
├── config/
│   └── amcl.yaml
└── rviz/
    └── localization.rviz
```

### `package.xml`

Defines package dependencies.

Important dependencies include:

```text
mapping_test
nav2_map_server
nav2_amcl
nav2_lifecycle_manager
rviz2
```

`mapping_test` is needed because the localization package uses the replay node from `MappingTest`.

### `setup.py`

Defines the Python package installation.

This package may not need many custom Python nodes if it mainly uses launch files and Nav2 nodes.

### `localization_test.launch.py`

This is the main localization launch file.

It starts:

```text
map_server
AMCL
lifecycle_manager_localization
replay_carmen_log
RViz
optional global localization service call
```

This file connects the saved map, replayed sensor data, and AMCL.

### `amcl.yaml`

This file contains the AMCL parameters.

Important parameters include:

```text
global_frame_id: map
odom_frame_id: odom
base_frame_id: base_footprint
scan_topic: /scan
set_initial_pose
initial_pose
min_particles
max_particles
laser model parameters
odometry noise parameters
```

### `localization.rviz`

RViz configuration used to visualize localization.

Useful displays include:

```text
Map
LaserScan
TF
PoseWithCovariance
ParticleCloud
Odometry
```

---

## Known Start and Unknown Start Localization

### Known Start

In known start localization, AMCL is given an initial pose.

This can be configured in `amcl.yaml`:

```yaml
set_initial_pose: true
initial_pose:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: -1.5707963268
```

This is useful when the starting location is already known.

### Unknown Start

In unknown start localization, the robot does not know its starting pose.

AMCL can use global localization by spreading particles across the map.

The global localization service is:

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}
```

During unknown start localization, it is normal to temporarily see messages such as:

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

A good sign is:

```text
Global initialisation done!
```

This means AMCL has started global localization.

---

## How LocalizationTest Connects to MappingTest

`LocalizationTest` connects to `MappingTest` in two ways:

```text
1. It uses the map files created by MappingTest.
2. It uses the replay_carmen_log node from MappingTest.
```

The connection is:

```text
MappingTest
   ↓ creates
my_map.pgm + my_map.yaml
   ↓ used by
LocalizationTest map_server
```

and:

```text
MappingTest replay_carmen_log.py
   ↓ used by
LocalizationTest
   ↓ publishes
/scan, /odom, /tf
```

Because of this, `MappingTest` must be built and sourced before `LocalizationTest`.

---

## How LocalizationTest Connects to NavigationTest

`NavigationTest` uses the output from `LocalizationTest`.

The connection is:

```text
LocalizationTest
   ↓ publishes
/map, /scan, /odom, /tf, /amcl_pose
   ↓ used by
NavigationTest
```

Navigation needs the estimated pose because the planner must know the current wheelchair location before it can create a path.

The most important data for navigation is the TF chain:

```text
map -> odom -> base_footprint
```

---

## Build Instructions

Go to the package folder:

```bash
cd /home/rudrh/Autonomous-Wheelchair-System/Positioning/Localization/LocalizationTest
```

Source ROS2 and MappingTest:

```bash
source /opt/ros/jazzy/setup.bash
source /home/rudrh/Autonomous-Wheelchair-System/Positioning/MapGeneration/MappingTest/install/setup.bash
```

Build the package:

```bash
colcon build \
  --packages-select localization_test \
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

Run localization:

```bash
ros2 launch localization_test localization_test.launch.py
```

Useful checks:

```bash
ros2 node list
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
ros2 run tf2_ros tf2_echo map base_footprint
```

Check lifecycle states:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Expected state:

```text
active [3]
```

---

## Summary

`LocalizationTest` loads the map created by `MappingTest`, replays sensor data, and runs AMCL to estimate the wheelchair pose.

Its most important output is:

```text
/amcl_pose
```

and the most important transform is:

```text
map -> odom
```

These outputs are required by `NavigationTest`.
