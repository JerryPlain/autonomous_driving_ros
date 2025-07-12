# Perception Module by Shijie Zhou

This package implements the perception pipeline for the autonomous driving project. It processes depth images from the Unity simulator, generates 3D point clouds, and builds an OctoMap-based occupancy grid for downstream modules like planning and control.

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

### Verify System is Working

#### Check topic publishing status
```bash
# Check all related topics
rostopic list | grep -E "(depth|octomap|map|Unity)"

# Should see these key topics:
# /depth/points                    - 3D point cloud data
# /projected_map                   - 2D occupancy grid map ⭐
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

#### Check TF transforms
```bash
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera
# Should see correct transform data (timestamp 0.000 is normal for static transforms)
```

### 📊 **RViz Visualization Setup**

```bash
rviz
```

Add the following displays in RViz:

1. **TF Display**
   - Add → By display type → TF
   - View complete coordinate frame tree

2. **3D Point Cloud Display**
   - Add → By topic → /depth/points → PointCloud2
   - View real-time depth point cloud

3. **2D Occupancy Grid Map Display** ⭐
   - Add → By display type → Map
   - Topic: /projected_map
   - View generated 2D navigation map

4. **OctoMap Voxel Display**
   - Add → By topic → /octomap_point_cloud_centers → PointCloud2
   - View 3D occupancy grid

5. **Set Reference Frame**
   - Global Options → Fixed Frame → OurCar/INS

### **Key Generated Outputs**

- **3D Point Cloud**: `/depth/points` (sensor_msgs/PointCloud2)
- **2D Occupancy Grid Map**: `/projected_map` (nav_msgs/OccupancyGrid) 
- **3D Voxel Map**: `/octomap_point_cloud_centers` (sensor_msgs/PointCloud2)
- **complete TF Tree**: world → OurCar/INS → Sensors

### **Configuration Details**

#### OctoMap 2D Projection Parameters
- `resolution: 0.1` - Map resolution 10cm
- `incremental_2D_projection: true` - Enable 2D projection
- `occupancy_min_z: -1.0, occupancy_max_z: 1.0` - 2D projection height range
- `pointcloud_min_z: -2.0, pointcloud_max_z: 2.0` - Point cloud processing height range

#### Static TF Transform Configuration
```xml
<!-- Use tf2_ros/static_transform_publisher for proper timestamp handling -->
<node pkg="tf2_ros" type="static_transform_publisher" name="tf_depth_camera"
      args="0.3 0.0 1.2 -1.5708 0 -1.5708 OurCar/INS OurCar/Sensors/DepthCamera" />
```

### **Downstream Applications**
The generated 2D occupancy grid map (`/projected_map`) can be directly used for:
- ROS Navigation Stack
- Path planning algorithms
- SLAM mapping
- Obstacle detection and avoidance

---

## Legacy Debugging Information (Issues Resolved)

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

### Original Project Structure

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

#### TF Verification Commands
```bash
# Verify single transform
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera

# Generate and view TF tree
rosrun tf view_frames
xdg-open frames.pdf
```

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

---

## **Quick Reference**

### Essential Commands
```bash
# Build and source
cd ~/autonomous_ws && catkin_make && source devel/setup.bash

# Launch sequence
roslaunch simulation simulation.launch    # Terminal 1
roslaunch perception perception.launch    # Terminal 2  
roslaunch perception octomap.launch      # Terminal 3
rviz                                     # Terminal 4

# Verification
rostopic hz /depth/points                # ~20-30Hz
rostopic info /projected_map             # nav_msgs/OccupancyGrid
rostopic list | grep -E "(depth|octomap|map)"
```

### Troubleshooting
- **No point cloud**: Check if Unity simulation is running and depth images are publishing
- **Empty OctoMap**: Verify TF transforms and point cloud frame_id
- **Nodelet errors**: Restart perception.launch, ensure no duplicate nodelet managers
- **RViz not showing map**: Set Fixed Frame to "OurCar/INS", check topic names

---

## 📚 **Git & GitHub Workflow**

### **Initial Setup (First Time Only)**

#### 1. Configure Git (if not done already)
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

#### 2. Initialize Git Repository
```bash
cd ~/autonomous_ws
git init
```

#### 3. Create .gitignore file
```bash
# Create .gitignore to exclude build files
cat > .gitignore << EOF
# ROS build files
build/
devel/
install/
.catkin_workspace

# Compiled files
*.pyc
*.so
*.o
*.a

# IDE files
.vscode/
.idea/
*.swp
*.swo

# OS files
.DS_Store
Thumbs.db

# Log files
*.log

# Temporary files
*~
*.tmp

# PDF files (can be regenerated)
frames.pdf
EOF
```

### **Daily Workflow: Committing Changes**

#### 1. Check Current Status
```bash
cd ~/autonomous_ws
git status
# Shows modified, added, deleted files
```

#### 2. Add Files to Staging
```bash
# Add specific files
git add src/perception/README_perception_english.md
git add src/perception/launch/perception.launch
git add src/perception/launch/octomap.launch

# Or add all modified files
git add .

# Or add all files in perception package
git add src/perception/
```

#### 3. Commit Changes
```bash
# Commit with descriptive message
git commit -m "feat: implement 2D occupancy grid mapping with OctoMap

- Fixed TF timestamp issues using tf2_ros/static_transform_publisher
- Resolved nodelet conflicts in launch files
- Added 2D projection parameters for navigation
- Updated README with complete workflow and troubleshooting"
```

#### 4. View Commit History
```bash
git log --oneline
# Shows recent commits
```

### **Connecting to GitHub**

#### 1. Create Repository on GitHub
1. Go to [github.com](https://github.com)
2. Click "New Repository" (green button)
3. Name: `autonomous_ws` or `autonomous_driving_perception`
4. Description: "ROS perception module for autonomous driving with Unity simulation"
5. Choose Public or Private
6. **Don't** initialize with README (you already have one)
7. Click "Create Repository"

#### 2. Connect Local Repository to GitHub
```bash
cd ~/autonomous_ws

# Add remote repository (replace YOUR_USERNAME and REPO_NAME)
git remote add origin https://github.com/YOUR_USERNAME/autonomous_ws.git

# Verify remote connection
git remote -v
```

#### 3. Push Code to GitHub
```bash
# Push to main branch (first time)
git branch -M main
git push -u origin main

# For subsequent pushes
git push origin main
```

### **Working with Team Members**

#### 1. Clone Repository (for team members)
```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/autonomous_ws.git
cd autonomous_ws

# Build the workspace
catkin_make
source devel/setup.bash
```

#### 2. Pull Latest Changes
```bash
cd ~/autonomous_ws
git pull origin main
# Gets latest changes from GitHub
```

#### 3. Create Feature Branches
```bash
# Create new branch for navigation work
git checkout -b feature/navigation-integration

# Work on navigation code...
# ... make changes ...

# Commit changes
git add .
git commit -m "feat: integrate 2D map with navigation stack"

# Push feature branch
git push origin feature/navigation-integration
```

#### 4. Create Pull Request
1. Go to your GitHub repository
2. Click "Compare & pull request" 
3. Add description of changes
4. Request review from team members
5. Merge after approval

### **Common Git Commands Quick Reference**

```bash
# Check status
git status

# View changes
git diff
git diff --staged

# Add files
git add filename.txt
git add .                    # Add all files
git add src/perception/      # Add specific folder

# Commit
git commit -m "descriptive message"
git commit -am "message"     # Add and commit modified files

# Push/Pull
git push origin main
git pull origin main

# Branching
git branch                   # List branches
git branch feature-name      # Create branch
git checkout feature-name    # Switch branch
git checkout -b feature-name # Create and switch

# Merge
git checkout main
git merge feature-name

# View history
git log
git log --oneline
git log --graph --oneline
```

### **Recommended Commit Message Format**

```bash
# Format: type(scope): description
# 
# Types:
# feat:     New feature
# fix:      Bug fix
# docs:     Documentation changes
# style:    Code style changes
# refactor: Code refactoring
# test:     Adding tests
# chore:    Build/tool changes

# Examples:
git commit -m "feat(perception): add 2D occupancy grid mapping"
git commit -m "fix(launch): resolve nodelet conflicts in octomap.launch"  
git commit -m "docs(readme): add GitHub workflow instructions"
git commit -m "refactor(tf): use tf2_ros static transform publisher"
```

### **Sharing with Navigation Team**

#### For Navigation Team Members:
```bash
# 1. Clone the repository
git clone https://github.com/YOUR_USERNAME/autonomous_ws.git
cd autonomous_ws

# 2. Install dependencies
sudo apt update
sudo apt install ros-noetic-nodelet ros-noetic-depth-image-proc \
                 ros-noetic-octomap-server ros-noetic-octomap-rviz-plugins

# 3. Build workspace
catkin_make
source devel/setup.bash

# 4. Test the perception system
roslaunch simulation simulation.launch    # Terminal 1
roslaunch perception perception.launch    # Terminal 2
roslaunch perception octomap.launch      # Terminal 3

# 5. Verify 2D map for navigation
rostopic info /projected_map              # Should show nav_msgs/OccupancyGrid
```

#### Key Information for Navigation Team:
- **2D Map Topic**: `/projected_map` (nav_msgs/OccupancyGrid)
- **Map Frame**: `OurCar/INS` 
- **Map Resolution**: 0.1m (10cm per pixel)
- **Update Rate**: Real-time (as robot moves)
- **Map Coordinate System**: OurCar/INS frame (robot-centric)

---
