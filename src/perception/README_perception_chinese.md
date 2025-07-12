# Perception Module - 自动驾驶感知系统

本模块实现了完整的自动驾驶感知管道，处理Unity仿真器的深度图像，生成3D点云，并构建基于OctoMap的占用栅格地图，为下游的规划和控制模块提供环境感知。

## **功能特性**

- ✅ **静态TF广播** - 传感器坐标系对齐 (`OurCar/INS` → 各传感器坐标系)
- ✅ **深度图像到点云转换** - 使用 `depth_image_proc` 处理Unity深度数据
- ✅ **3D OctoMap占用地图** - 实时构建3D环境地图
- ✅ **2D占用栅格地图生成** - 为导航提供2D地图 (`nav_msgs/OccupancyGrid`)
- ✅ **ROS Launch文件** - 一键启动所有模块
- ✅ **Unity兼容** - 话题重映射支持Unity仿真器

## **系统架构**

```
Unity仿真器 → 深度图像 → 点云转换 → OctoMap → 2D/3D占用地图
     ↓              ↓           ↓          ↓
  传感器数据    depth_image_proc   3D建图    导航地图
```

## **传感器配置**

- **1个深度相机** - 主要建图传感器
- **2个RGB相机** (左/右) - 视觉感知  
- **1个语义相机** - 语义分割
- **1个惯性导航系统(INS)** - 基准坐标系

## **依赖包安装**

确保安装以下ROS包：

```bash
sudo apt update
sudo apt install \
  ros-noetic-nodelet \
  ros-noetic-depth-image-proc \
  ros-noetic-octomap-server \
  ros-noetic-octomap-rviz-plugins \
  ros-noetic-tf2-ros
```

## **问题解决方案和完整工作流程**

### **解决的关键问题**
1. **TF变换时间戳问题** - 使用 `tf2_ros/static_transform_publisher` 替代自定义TF广播器
2. **Nodelet冲突** - 避免在perception.launch和octomap.launch中重复创建nodelet
3. **2D地图配置** - 正确配置OctoMap参数以生成2D占用栅格地图

### **完整启动步骤（已验证有效）**

#### Step 1: 编译工作空间
```bash
cd ~/autonomous_ws
catkin_make
source devel/setup.bash
```

#### Step 2: 启动Unity仿真环境
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch simulation simulation.launch
```

#### Step 3: 启动感知模块（TF + 点云生成）
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception perception.launch
```

#### Step 4: 启动OctoMap（3D地图 + 2D占用栅格地图）
```bash
cd ~/autonomous_ws
source devel/setup.bash
roslaunch perception octomap.launch
```

### **验证系统正常工作**

#### 检查话题发布状态
```bash
# 检查所有相关话题
rostopic list | grep -E "(depth|octomap|map|Unity)"

# 应该看到以下关键话题：
# /depth/points                    - 3D点云数据
# /projected_map                   - 2D占用栅格地图 ⭐
# /octomap_point_cloud_centers     - OctoMap体素中心
# /Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw
```

#### 检查点云生成频率
```bash
rostopic hz /depth/points
# 应该看到约20-30Hz的发布频率
```

#### 检查2D占用栅格地图
```bash
rostopic info /projected_map
# Type: nav_msgs/OccupancyGrid

rostopic echo /projected_map -n1 | head -10
# 应该看到有效的header和map数据
```

#### 检查TF变换
```bash
rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera
# 应该看到正确的变换数据（时间戳为0.000是静态变换的正常现象）

# 生成TF树PDF
rosrun tf view_frames
xdg-open frames.pdf
```

### **RViz可视化设置**

```bash
rviz
```

在RViz中添加以下显示：

1. **设置参考坐标系**
   - Global Options → Fixed Frame → `OurCar/INS`

2. **TF显示**
   - Add → By display type → TF
   - 查看完整的坐标系树

3. **3D点云显示**
   - Add → By topic → `/depth/points` → PointCloud2
   - 查看实时深度点云

4. **2D占用栅格地图显示** ⭐
   - Add → By display type → Map
   - Topic: `/projected_map`
   - 查看生成的2D导航地图

5. **OctoMap体素显示**
   - Add → By topic → `/octomap_point_cloud_centers` → PointCloud2
   - 查看3D占用栅格

### **生成的关键输出**

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/depth/points` | sensor_msgs/PointCloud2 | 3D点云数据 |
| `/projected_map` | nav_msgs/OccupancyGrid | 2D占用栅格地图 |
| `/octomap_point_cloud_centers` | sensor_msgs/PointCloud2 | 3D体素地图 |
| `/octomap_binary` | octomap_msgs/Octomap | 二进制OctoMap |
| `/octomap_full` | octomap_msgs/Octomap | 完整OctoMap |

### **配置说明**

#### OctoMap 2D投影参数
```xml
<param name="resolution" value="0.1"/>                    <!-- 地图分辨率10cm -->
<param name="incremental_2D_projection" value="true"/>    <!-- 启用2D投影 -->
<param name="occupancy_min_z" value="-1.0"/>             <!-- 2D投影最低高度 -->
<param name="occupancy_max_z" value="1.0"/>              <!-- 2D投影最高高度 -->
<param name="pointcloud_min_z" value="-2.0"/>            <!-- 点云处理最低高度 -->
<param name="pointcloud_max_z" value="2.0"/>             <!-- 点云处理最高高度 -->
```

#### 静态TF变换配置
```xml
<!-- 使用tf2_ros/static_transform_publisher确保正确的时间戳处理 -->
<node pkg="tf2_ros" type="static_transform_publisher" name="tf_depth_camera"
      args="0.3 0.0 1.2 -1.5708 0 -1.5708 OurCar/INS OurCar/Sensors/DepthCamera" />
```

### **下游应用**
生成的2D占用栅格地图(`/projected_map`)可直接用于：
- **ROS Navigation Stack** - 标准导航框架
- **路径规划算法** - A*、RRT等算法
- **SLAM建图** - 同步定位与建图
- **障碍物检测与避障** - 实时避障系统

---

## **任务详细说明**

### 任务1：坐标系构建 (Coordinate Frame Build-up)

**目标**：建立从世界坐标系到各个传感器的完整TF树

**实现**：
- 世界坐标系到 `OurCar/INS` 的变换由Unity仿真器提供
- 静态变换从 `OurCar/INS` 到各个前置摄像头：
  - 深度相机 (DepthCamera) - [0.3, 0.0, 1.2]
  - 左RGB相机 (RGBCameraLeft) - [0.3, 0.1, 1.2]
  - 右RGB相机 (RGBCameraRight) - [0.3, -0.1, 1.2]
  - 语义相机 (SemanticCamera) - [0.3, 0.0, 1.3]

**坐标系层次结构**：
```
world
└── OurCar/INS
    ├── OurCar/Sensors/DepthCamera
    ├── OurCar/Sensors/RGBCameraLeft
    ├── OurCar/Sensors/RGBCameraRight
    └── OurCar/Sensors/SemanticCamera
```

### 任务2：占用栅格地图生成 (Occupancy Map Generation)

**目标**：将深度图像转换为3D点云，然后生成OctoMap占用栅格地图

**处理流程**：
1. Unity深度图像 (`/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw`)
2. depth_image_proc转换为3D点云 (`/depth/points`)
3. OctoMap构建3D占用地图
4. 投影生成2D占用栅格地图 (`/projected_map`)

---

## **故障排除**

### 常见问题及解决方案

1. **"octree is empty"错误**
   - 确认点云正在发布：`rostopic hz /depth/points`
   - 检查TF变换可用：`rosrun tf tf_echo OurCar/INS OurCar/Sensors/DepthCamera`
   - 重启感知模块：先停止再启动perception.launch

2. **nodelet加载失败**
   - 清理nodelet进程：`pkill -f nodelet`
   - 重新启动launch文件

3. **TF时间戳为0.000**
   - 这是静态变换的正常现象，不影响功能
   - OctoMap能正确处理静态变换

4. **点云不发布**
   - 检查Unity仿真器是否运行
   - 确认深度图像发布：`rostopic hz /Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw`
   - 重启perception.launch

### 系统要求
- Ubuntu 20.04 + ROS Noetic
- 充足内存（建议8GB+）用于OctoMap处理
- Unity仿真器正常运行

---

## **学习资源**

- [OctoMap官方文档](https://octomap.github.io/)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [depth_image_proc包文档](http://wiki.ros.org/depth_image_proc)
- [TF2教程](http://wiki.ros.org/tf2/Tutorials)

---

*作者：Shijie Zhou | 更新：解决了TF时间戳和2D地图生成问题*
