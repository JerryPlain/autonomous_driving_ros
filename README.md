# autonomous_driving_ros

A ROS Noetic workspace for a Unity-based autonomous driving stack, including simulation bridge, perception, traffic-light understanding, high-level decision making, low-level control, and trajectory tooling.

This repository is organized as a multi-package Catkin workspace intended for rapid integration experiments and module-level iteration.

## System Overview

The runtime pipeline forms a closed loop:

1. Unity publishes vehicle state and sensor streams (RGB, semantic, depth, IMU).
2. `simulation` ingests the stream, republishes ROS topics, and broadcasts vehicle TF.
3. Perception and traffic-light modules process camera/depth data.
4. `decision_making` converts traffic-light state into driving intent.
5. `dummy_controller` (PID-based controller) fuses goals + vehicle state + intent into control commands.
6. `simulation` transmits commands back to Unity via UDP.

## Repository Layout

```text
.
├── src/
│   ├── simulation/             # Unity <-> ROS bridge (TCP Rx, UDP Tx, config loader)
│   ├── perception/             # Depth->PointCloud + OctoMap projection tooling
│   ├── traffic_light_detector/ # Semantic/depth/RGB traffic light detection pipeline
│   ├── decision_making/        # Traffic light color -> driving decision
│   ├── dummy_controller/       # PID tracking controller + traffic override
│   ├── path_recorder/          # Path recording, replay, and visualization utilities
│   └── libsocket/              # Socket dependency used by simulation bridge
├── Final_Report.pdf
├── Final_Presentation.pptx
└── README.md
```

## Core Packages

### `simulation`
- Loads runtime network parameters from Unity JSON config.
- Receives Unity TCP sensor/state stream and publishes ROS topics.
- Broadcasts `world -> OurCar/INS` transform from ground-truth state.
- Sends `simulation/VehicleControl` commands to Unity over UDP.

Key launch files:
- `src/simulation/launch/simulation.launch`
- `src/simulation/launch/simulation_no_Unity.launch`

### `perception`
- Provides static sensor transforms from vehicle body frame.
- Converts depth images to point cloud (`/depth/points`) via `depth_image_proc`.
- Builds OctoMap and projects to 2D occupancy grid (`/projected_map`).

Key launch files:
- `src/perception/launch/perception.launch`
- `src/perception/launch/octomap.launch`

### `traffic_light_detector`
Three-stage traffic-light pipeline:
- semantic + depth bounding boxes
- depth-to-RGB projection
- HSV color classification

Primary output:
- `traffic_light_color` (`traffic_light_detector/TrafficLightColor`)

### `decision_making`
- Maps traffic-light color to high-level behavior labels:
  - `red -> BRAKE`
  - `yellow -> ACCELERATE`
  - `green -> DRIVE`
- Publishes periodic `traffic_decision` with timeout fallback logic.

### `dummy_controller`
- PID controller that tracks `/move_base_simple/goal`.
- Uses vehicle pose/twist feedback from Unity bridge.
- Applies traffic decision override (hard brake on `BRAKE`).
- Publishes:
  - `car_command` (`simulation/VehicleControl`)
  - `/reached_goal` (`std_msgs/Int32`)

### `path_recorder`
Unified path utility package with normalized CSV schema:

```text
index,timestamp,x,y,z,yaw,velocity
```

Tools include:
- sparse waypoint recording from pose/twist
- CSV goal replay
- path and point visualization
- static TF waypoint publication

## Key Runtime Topics

- `/Unity_ROS_message_Rx/OurCar/CoM/pose` (`geometry_msgs/PoseStamped`)
- `/Unity_ROS_message_Rx/OurCar/CoM/twist` (`geometry_msgs/TwistStamped`)
- `/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw` (`sensor_msgs/Image`)
- `car_command` (`simulation/VehicleControl`)
- `traffic_light_color` (`traffic_light_detector/TrafficLightColor`)
- `traffic_decision` (`std_msgs/String`)
- `/depth/points` (`sensor_msgs/PointCloud2`)
- `/projected_map` (`nav_msgs/OccupancyGrid`)

## Prerequisites

- Ubuntu with ROS Noetic
- Catkin workspace tooling (`catkin_tools`)
- Core ROS dependencies used by this workspace (e.g. `tf2_ros`, `nodelet`, `depth_image_proc`, `octomap_server`, `cv_bridge`)

Example installation:

```bash
sudo apt update
sudo apt install -y \
  python3-catkin-tools \
  ros-noetic-tf2-ros \
  ros-noetic-nodelet \
  ros-noetic-depth-image-proc \
  ros-noetic-octomap-server \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport
```

## Build

From repository root:

```bash
chmod +x src/setup_script.sh
./src/setup_script.sh
catkin build
source devel/setup.bash
```

## Recommended Bring-Up Sequence

1. Start simulation bridge (with Unity process):

```bash
source devel/setup.bash
roslaunch simulation simulation.launch
```

2. Start traffic-light detection pipeline:

```bash
source devel/setup.bash
roslaunch traffic_light_detector detection_pipeline.launch
```

3. Start decision node:

```bash
source devel/setup.bash
rosrun decision_making decision_making_node
```

4. Start controller node:

```bash
source devel/setup.bash
rosrun dummy_controller dummy_controller_node
```

5. (Optional) Replay goals from CSV:

```bash
source devel/setup.bash
rosrun path_recorder csv_goal_publisher
```

6. (Optional) Start mapping pipeline:

```bash
source devel/setup.bash
roslaunch perception perception.launch
roslaunch perception octomap.launch
```

## Configuration Notes

- Several nodes are parameterized via private ROS params (`~param_name`), especially in `path_recorder`, `decision_making`, `dummy_controller`, and perception helper nodes.
- Unity network and sensor settings are read from:
  - `src/simulation/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json`

## Known Limitations

- This is still a research/course-style integration codebase, not a production-hardened stack.
- Some legacy artifacts remain in non-critical files and documentation from earlier iterations.
- Full-system behavior depends on Unity build compatibility and local machine runtime setup.

## Documentation

- Chinese project analysis and summary:
  - `docs/项目总结与解析.md`
