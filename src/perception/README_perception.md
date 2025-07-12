# Perception Module by Shijie Zhou

This package implements the perception pipeline for the autonomous driving project. It processes depth images from the Unity simulator, generates 3D point clouds, and builds an OctoMap-based occupancy grid for downstream modules like planning and control.


## Pipeline

- Static TF broadcast for sensor-frame alignment (`base_link` → `camera_depth_frame`)
- Depth image to point cloud conversion via `depth_image_proc`
- 3D OctoMap occupancy mapping from point cloud input
- ROS launch files for easy module startup
- Topic remapping for Unity-compatible camera inputs


## Dependencies

Make sure the following ROS packages are installed:

```bash
sudo apt update
sudo apt install \
  ros-noetic-nodelet \
  ros-noetic-depth-image-proc \
  ros-noetic-octomap-server \
  ros-noetic-octomap-rviz-plugins
```
Steps to start the perception package:
- clone this repo
```bash
cd ~/<git_repository>
catkin build
source devel/setup.bash
```
Launch two files and the simulation
```bash
roslaunch simulation simulation.launch
roslaunch perception perception.launch 
roslaunch perception octomap.launch
```
### For debugging / Visualization
Open Rviz
```bash
rviz
```
Choose following topics:
- "Add" -> "By topic" -> "/octomap_point_cloud_centers" -> "PointCloud2"
- "Add" -> "By display type" -> "TF"
- "Global Options" -> "Fixed Frame" -> "Our_Car/INS
- "PointCloud2" -> "Topic" -> "/depth/points" 
  

Perception Task 1 – Coordinate Frame Build-up
Description
This module builds the full static TF tree between the world coordinate frame and each onboard sensor. The transformation from world to OurCar/INS is provided by the simulation. We are responsible for broadcasting static transformations from OurCar/INS to each of the front-mounted cameras (depth, RGB left/right, semantic).
How to Run
#### Build the workspace
cd ~/autonomous_ws
catkin_make
source devel/setup.bash

#### Launch the simulation
source devel/setup.bash
roslaunch simulation simulation.launch

#### Launch the perception system (TF broadcaster + DepthCamera pipeline)
# This launches:
# 4 static TF broadcasters (OurCar/INS → DepthCamera, etc.)
# nodelet manager
# depth image to point cloud converter
source devel/setup.bash
roslaunch perception perception.launch

#### How to Verify TF Tree
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera

rosrun tf view_frames
evince frames.pdf  # or xdg-open frames.pdf
# you should see a fully connected tree like this:
world
└── OurCar/INS
    ├── OurCar/Sensors/DepthCamera
    ├── OurCar/Sensors/RGBCameraLeft
    ├── OurCar/Sensors/RGBCameraRight
    └── OurCar/Sensors/SemanticCamera


## documentation of the task 1
Static Transform Broadcasters
We publish static transforms from the OurCar/INS coordinate frame (provided by Unity) to the four front-mounted sensors using static_transform_publisher. This builds a complete TF tree for perception and planning.
TF Node setting:
All frames are relative to OurCar/INS and use an identity rotation (no tilt). Translation values are estimated from vehicle dimensions and camera placements in Unity.

Frame ID Alignment
Each camera publishes messages on /image_raw and /camera_info, where the header.frame_id must match the child_frame_id used in the static transform.
Example: /Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw → frame_id = OurCar/Sensors/DepthCamera
Therefore: static_transform_publisher must also use OurCar/Sensors/DepthCamera

frames.pdf
Attached below is the generated TF tree using rosrun tf view_frames.
It shows the full connection from world → OurCar/INS → Sensors.

Verification
1. Verify single transform with tf_echo
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera
2. Verify full tree with tf view_frames
rosrun tf view_frames
xdg-open frames.pdf

Visualize in RViz
Rviz add TF to see the tree structure
setup the fixed frame to be OurCar/INS (It has to be the same as the connected point in TF Tree)
Add TF, then we can see the world -> OurCar/INS -> Sensors



































# Instructions for launching the peception pipeline:

###### Coordinate frame build-up

# Step One: Clean and compile the workspace.
cd ~/autonomous_ws
catkin_make
source devel/setup.bash

# Step two: setup unity simulation environment
cd ~/autonomous_ws
source devel/setup.bash
roslaunch simulation simulation.launch

# Step three: launch the perception
catkin_make
source devel/setup.bash
roslaunch perception perception.launch


# check:
rostopic list
# you should see this: it means all the sensor topics are published
/Unity_ROS_message_Rx/OurCar/CoM/pose
/Unity_ROS_message_Rx/OurCar/CoM/twist
/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info
/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw
/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/camera_info
/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw
/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraRight/camera_info
/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraRight/image_raw
/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/camera_info
/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw
/car_command
/depth/points
/depth_image_proc_manager/bond
/rosout
/rosout_agg
/tf
/tf_static

# check:
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera
# you should see this: It means that I have sucecssfully broadcast the transform from Ourcar/INS to OurCar/Sensors/DepthCamera
Translation: [0.300, 0.000, 1.200]
Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]

BUT, the time is always zero, which is why the octomap cannot take this as the input


###### Occupancy Map Generation for navigation

# Step three: activate the perception module.
# 3.1 start TF broadcaster + point cloud generation
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception perception.launch

# 3.2 test if the point cloud is generated
rostopic echo /depth/points | grep frame_id

# you should see this:
frame_id: "OurCar/Sensors/DepthCamera" # the depth image from unity has been converted to point cloud

# Step four: Use OctoMap to build a 3D map. We need to start the OctoMap Server, which will subscribe to /depth/points and construct the octree map
# 4.1 run OctoMap
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception octomap.launch

# you should see this: 
[INFO]: Publishing latched...
[WARN]: Nothing to publish, octree is empty

# check if the point cloud is launched
rostopic echo /depth/points

# check point cloud header.frame_id is connected to TF
rostopic echo -n1 /depth/points | grep frame_id
# check if Octomap can find the path of TF
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera
# Make sure that the data is not empty point cloud
rostopic echo -n1 /depth/points
# check if Octomap can successfully TF transform
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera

rviz


# 你应该广播：
# /INS → /OurCar/Sensors/DepthCamera
# 然后 /depth/points 默认的 frame_id 要等于 /OurCar/Sensors/DepthCamera
# 你现在用 OurCar/INS 作为建图基坐标系，所以你必须能从点云的 header.frame_id 变换到 OurCar/INS。


# /depth/points 发布的是一个格式不完整的 PointCloud2 消息，缺少 header、width、height 等关键信息，OctoMap 因此无法使用它构建地图，导致 “octree is empty”。
1. depth_image_proc/point_cloud_xyz 没有正确加载为 Nodelet
2. 深度图或相机信息为空或不规范
