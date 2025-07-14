# autonomous_driving_ros - Trajectory and Path Visualization

## ✅ Overview

This package provides:

- ✅ Displaying trajectory points recorded in a CSV file as **Marker points** in RViz
- ✅ Displaying **Path** (nav_msgs/Path) in RViz
- ✅ Running and visualizing your **saved static map (project map)**

---

## 📄 Folder and file preparation

### Map files

Place your saved `.pgm` and `.yaml` files together, for example:

maps/
├── my_map.pgm
└── my_map.yaml

yaml

---

### CSV file

Points recorded every 0.5 seconds, example CSV structure:

time,x,y,v
1689773300.23,1.2,0.5,0.3
1689773300.73,1.4,0.7,0.3
...

yaml


---

## 🌍 Load your saved map

### Command

```bash
rosrun map_server map_server /home/user/maps/my_map.yaml
⚠️ Important:
Replace /home/user/maps/my_map.yaml with your actual absolute path.

📍 Display trajectory points (Markers)


rosrun path_recorder showpoints
Reads recorded points from the CSV file (configured inside showpoints.cpp)

Visualizes them as Marker points (small spheres) in RViz

🟢 Display path


rosrun path_recorder show_path_node
Publishes and displays the planned path (nav_msgs/Path) in RViz

🗺️ RViz configuration
Fixed Frame: map

Add "Map" display (topic: /map)

Add "Marker" display (topic: /visualization_marker)

Add "Path" display (topic: /path)

(Optional) Add "RobotModel" to visualize the robot

⚠️ Notes
The CSV file path must be set correctly in showpoints.cpp

The map YAML path must use an absolute path, for example: /home/user/maps/my_map.yaml

If you change file names or paths, also update the corresponding code or command

✅ Summary
mathematica

✅ Only three steps:
1️⃣ Load your saved map
2️⃣ Display CSV trajectory points (Markers)
3️⃣ Display path (Path)

You can then visualize the trajectory points and planned path together in RViz!
