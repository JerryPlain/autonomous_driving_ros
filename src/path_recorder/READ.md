# 🧭 Path Recorder for ROS

This ROS package continuously records the trajectory of a robot or autonomous vehicle and provides tools to visualize and export the recorded path. It is useful for debugging, analysis, and visualization in navigation and motion planning scenarios.

---

## 🚗 Features

- ✅ Subscribes to pose or odometry to record real-time trajectories.
- ✅ Saves the full path to a CSV file (`recorded_path.csv`).
- ✅ Publishes the path as `nav_msgs/Path`.
- ✅ Provides RViz visualization via `Path` and/or `MarkerArray`.

---

## 🔧 Build Instructions

Make sure the package is located inside your `catkin_ws` workspace:

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
▶️ Run the Path Recorder
Start the main recorder node:


rosrun path_recorder path_recorder_node
This node logs the robot’s path in real time and writes to a CSV file.

🔁 (Optional) Path Playback & Visualization
To re-publish or visualize the recorded path:


rosrun path_recorder path_publisher_node
rosrun path_recorder show_path_node
📁 CSV Output Format (recorded_path.csv)
Each row contains:

csv

timestamp, x, y, z, yaw, velocity
This format may vary depending on the node implementation and subscribed message type.

🧪 RViz Visualization Setup
To view the path in RViz, subscribe to the following topics:

Path

Topic: /recorded_path

Type: nav_msgs/Path

Markers (optional)

Topic: /path_markers

Type: visualization_msgs/MarkerArray


