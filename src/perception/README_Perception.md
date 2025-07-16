# Perception Module by Shijie Zhou
Overview: Perception Module for Autonomous Driving
This module implements a complete perception pipeline for our ROS-based autonomous driving system with Unity simulation. It receives depth images from a virtual camera, generates 3D point clouds, constructs a 3D OctoMap, and projects it into a 2D occupancy grid (/projected_map) for downstream planning and navigation.

The key contributions of this module include:

✅ Coordinate Frame Setup: Built a full TF tree using static_transform_publisher to connect vehicle (OurCar/INS) with onboard sensors (Depth, RGB, Semantic cameras).

✅ Depth-to-PointCloud Conversion: Converted raw depth images from Unity into ROS-compliant point clouds (/depth/points) using depth_image_proc/point_cloud_xyz via nodelets.

✅ 3D + 2D Map Construction: Integrated octomap_server to build 3D voxel maps and enabled real-time 2D projection with height filtering.

✅ TF Timestamp Fix: Resolved OctoMap synchronization issues by replacing hand-written TF broadcasters with tf2’s static publishers (with valid timestamps).

✅ RViz Visualization: Provided recommended visualization setup for TF, 3D point cloud, OctoMap, and 2D occupancy grid.

✅ System Robustness: Resolved nodelet manager conflicts, ensured frame alignment, and tuned OctoMap parameters for stable map generation.

✅ Team Integration Ready: Produced /projected_map in nav_msgs/OccupancyGrid format with real-time updates and vehicle-centric frame, directly usable by the navigation stack.

This module is verified to work in real time with the Unity simulator and is ready to be integrated with motion planning, SLAM, or control modules.


## Working Solution & Complete Workflow

### Key Issues Resolved
1. **TF Transform Timestamp Issue** - Use `tf2_ros/static_transform_publisher` instead of custom TF broadcaster
2. **Nodelet Conflicts** - Avoid duplicate nodelet creation in perception.launch and octomap.launch
3. **2D Map Configuration** - Properly configure OctoMap parameters for 2D occupancy grid generation

### Complete Launch Steps (Verified Working)

#### Step 1: Build the workspace
```bash
cd ~/autonomous_ws
catkin_make
source devel/setup.bash
```

#### Step 2: Launch Unity simulation environment
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch simulation simulation.launch
```

#### Step 3: Launch perception module (TF + Point cloud generation)
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception perception.launch
```

#### Step 4: Launch OctoMap (3D map + 2D occupancy grid)
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception octomap.launch
```

### Step 5: Outcomes: TF Trees & Projected map in rviz visualization
Add the following displays in RViz:

1. **TF Display**
   - Add → By display type → TF
   - View complete coordinate frame tree

#### TF Verification Commands
```bash
# Verify single transform
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera

# Generate and view TF tree
rosrun tf view_frames
xdg-open frames.pdf
```

2. **3D Point Cloud Display**
   - Add → By topic → /depth/points → PointCloud2
   - View real-time depth point cloud

3. **2D Occupancy Grid Map Display** 
   - Add → By display type → Map
   - Topic: /projected_map
   - View generated 2D navigation map

4. **OctoMap Voxel Display**
   - Add → By topic → /octomap_point_cloud_centers → PointCloud2
   - View 3D occupancy grid

5. **Set Reference Frame**
   - Global Options → Fixed Frame → World

Run the car through the correct path to get the final projected map which can be seen in the map folder

### Verify System is Working
#### Check topic publishing status
```bash
# Check all related topics
rostopic list | grep -E "(depth|octomap|map|Unity)"

# Should see these key topics:
# /depth/points                    - 3D point cloud data
# /projected_map                   - 2D occupancy grid map (the projected map)
# /octomap_point_cloud_centers     - OctoMap voxel centers
# /Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw
```

#### Check point cloud generation rate
```bash
rostopic hz /depth/points
# Should see ~20-30Hz publishing rate
```

#### Check 2D occupancy grid map
```bash
rostopic info /projected_map
# Type: nav_msgs/OccupancyGrid

rostopic echo /projected_map -n1 | head -10
# Should see valid header and map data
```
### Key Generated Outputs

- **3D Point Cloud**: `/depth/points` (sensor_msgs/PointCloud2)
- **2D Occupancy Grid Map**: `/projected_map` (nav_msgs/OccupancyGrid) 
- **3D Voxel Map**: `/octomap_point_cloud_centers` (sensor_msgs/PointCloud2)
- **complete TF Tree**: world → OurCar/INS → Sensors

### **Downstream Applications**
The generated 2D occupancy grid map (`/projected_map`) can be directly used for:
- ROS Navigation Stack
- Path planning algorithms
- SLAM mapping
- Obstacle detection and avoidance

### Dependencies

Make sure the following ROS packages are installed:

```bash
sudo apt update
sudo apt install \
  ros-noetic-nodelet \
  ros-noetic-depth-image-proc \
  ros-noetic-octomap-server \
  ros-noetic-octomap-rviz-plugins
```


#### Perception Task 1 – Coordinate Frame Build-up
**Description**: This module builds the full static TF tree between the world coordinate frame and each onboard sensor. The transformation from world to OurCar/INS is provided by the simulation. We are responsible for broadcasting static transformations from OurCar/INS to each of the front-mounted cameras (depth, RGB left/right, semantic).

#### Expected TF Tree Structure
```
world
└── OurCar/INS
    ├── OurCar/Sensors/DepthCamera
    ├── OurCar/Sensors/RGBCameraLeft
    ├── OurCar/Sensors/RGBCameraRight
    └── OurCar/Sensors/SemanticCamera
```
#### Frame ID Alignment
Each camera publishes messages on `/image_raw` and `/camera_info`, where the `header.frame_id` must match the `child_frame_id` used in the static transform.

Example: `/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw` → `frame_id = OurCar/Sensors/DepthCamera`


### Previous Issues Encountered (Now Fixed)

1. **TF Timestamp Problem**: Static transforms showing `time: 0.000` caused OctoMap to fail
   - **Solution**: Using `tf2_ros/static_transform_publisher` resolves timestamp handling

2. **Empty Point Cloud**: `/depth/points` missing header, width, height information
   - **Solution**: Proper nodelet configuration in launch files

3. **Nodelet Conflicts**: Multiple nodelet managers causing crashes
   - **Solution**: Single nodelet manager in perception.launch, OctoMap-only in octomap.launch

4. **Empty OctoMap**: "Nothing to publish, octree is empty" warning
   - **Solution**: Proper TF chain and 2D projection parameters

### Legacy Debugging Commands (For Reference)
```bash
# Check if point cloud has correct frame_id
rostopic echo -n1 /depth/points | grep frame_id

# Verify depth camera image publishing
rostopic hz /Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw

# Check OctoMap status
rostopic echo /octomap_binary -n1

# Expected topic list after successful launch
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
/octomap_binary
/octomap_full
/octomap_point_cloud_centers
/projected_map
```

--

Hi everyone, I’m Shijie Zhou. In this project, I was responsible for designing and implementing the perception module of our autonomous driving system in ROS, which processes simulated depth images from Unity to generate both 3D and 2D maps for downstream use like planning and navigation.

#### 1. Static TF Tree Construction
I first built the complete TF coordinate frame tree for the car and its sensors.
Using tf2_ros/static_transform_publisher, I published static transforms from OurCar/INS to each sensor—DepthCamera, RGBCameraLeft, RGBCameraRight, and SemanticCamera—with hardcoded positions and Euler angles.

This resolved the timestamp issue we had with hand-written TF broadcasters, which caused OctoMap to reject incoming point clouds due to missing or zero timestamps.
(because OctoMap should receive point cloud, these messages should have clear header.stamp & header.frame_id. In order to transform the point cloud to world coordinate, Octomap will check the transform by tf. so it has to get the transform(from=pointcloud.frame_id, to=map_frame, at=stamp) from TF tree). The handwritten static TF broadcaster is just publish once, the timestamp will never update so the checking process of octomap will fail, that is the reason that we cannot get the octomap early.
But the tf2_ros/static_transform_publisher will provide a permanent static transform that every subscriber can get.

#### 2. Depth-to-PointCloud Conversion via Nodelet
The Unity will publish topic of two sensors
- /image_raw which is the 16-bit grey depth image
- /camera_info which is the interior parameters matrix

What I do is to use depth_image_proc/point_cloud_xyz nodelet
which is provided by ROS and used for turning the depth image & camera parameter martix to calculate the 3D point of each pixel and publish it as the sensor_msgs/PointCloud2

#### 3. OctoMap + 2D Projection Setup
After generating the point cloud from nodelet, i used the octomap_server_node which provided by ROS for creating the 3D pointcloud to OctoMap.
I remapped the input (by defaultl /cloud_in )to /depth/points.
To support navigation, I enabled incremental_2D_projection = true
which means, the OctoMap will transfrom the 3D map to 2D ocuupancy grid realtime. which can be used for navigation.
Height filters like occupancy_min_z and occupancy_max_z
so that we could extract a clean, robot-centric 2D occupancy grid from the 3D map.

#### 4. Debugging and Validation
I solved multiple issues:
Fixed missing transforms that caused empty OctoMaps
Avoided nodelet manager conflicts between perception and OctoMap
Verified everything in RViz: TF tree, point cloud, 3D voxel map, and the final 2D grid
Now the system runs stably with Unity in real time and produces a clean 2D occupancy map aligned with the robot frame.