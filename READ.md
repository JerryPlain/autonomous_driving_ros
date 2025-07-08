# Waypoint Logger for ROS

This ROS package detects turning points (left/right) of an autonomous vehicle, logs them with position and velocity, and visualizes the path and turns in RViz.

---

## 🚗 Features

- ✅ Detect left/right turns based on heading angle change.
- ✅ Record turning point coordinates and velocity to a CSV file.
- ✅ Publish the trajectory using `nav_msgs/Path`.
- ✅ Publish turning points as RViz markers using `visualization_msgs/MarkerArray`.
- #Mention! You should look at Jerry's branch first and operate with his description!
---

## 📁 CSV Format (`waypoints.csv`)

The data file includes six columns:

```csv
x, y, z, vx, vy, vz
x, y, z: Position of the vehicle at the turning point.

vx, vy, vz: Linear velocity of the vehicle at that moment.

Build Instructions
Make sure the package is inside your catkin workspace.

cd ~/catkin_ws
catkin build
source devel/setup.bash

Run the waypoint logger node

rosrun waypoint_logger waypoint_logger_fixed_node
This node will log detected turns into a waypoints.csv file and publish the full path to /logged_path.

Visualize waypoints and path in RViz

rosrun waypoint_logger show_waypoints_fixed_node
rosrun waypoint_logger path_publisher_fixed_node

rosrun waypoint_logger waypoint_logger_node
This node will log detected turns into a waypoints.csv file and publish the full path to /logged_path.

Visualize waypoints in RViz

rosrun waypoint_logger show_waypoints_node

Make sure RViz is subscribed to the following:

Path:

Topic: /logged_path

Type: nav_msgs/Path

Markers:

Topic: /waypoint_markers

Type: visualization_msgs/MarkerArray

